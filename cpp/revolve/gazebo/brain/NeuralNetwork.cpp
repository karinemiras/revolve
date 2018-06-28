/*
* Copyright (C) 2017 Vrije Universiteit Amsterdam
*
* Licensed under the Apache License, Version 2.0 (the "License");
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* Description: TODO: <Add brief description about file purpose>
* Author: Elte Hupkes
*
*/

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "../motors/Motor.h"
#include "../sensors/Sensor.h"

#include "NeuralNetwork.h"

namespace gz = gazebo;

namespace revolve {
namespace gazebo {

// Internal helper function to build neuron params
void neuronHelper(double* params, unsigned int* types, sdf::ElementPtr neuron);
void neuronHelper(double* params, unsigned int* types, const revolve::msgs::Neuron & neuron);

NeuralNetwork::NeuralNetwork(std::string modelName,
                             sdf::ElementPtr node,
                             std::vector< MotorPtr > & motors,
                             std::vector< SensorPtr > & sensors)
        : flipState_(false)
        , nInputs_(0)
        , nOutputs_(0)
        , nHidden_(0)
        , nNonInputs_(0)
{
  // Create transport node
  node_.reset(new gz::transport::Node());
  node_->Init();

  // Listen to network modification requests
  alterSub_ = node_->Subscribe("~/"+modelName+"/modify_neural_network",
                               &NeuralNetwork::modify, this);

  // Initialize weights, input and states to zero by default
  memset(inputWeights_, 0, sizeof(inputWeights_));
  memset(outputWeights_, 0, sizeof(outputWeights_));
  memset(hiddenWeights_, 0, sizeof(hiddenWeights_));
  memset(state1_, 0, sizeof(state1_));
  memset(state2_, 0, sizeof(state2_));
  memset(input_, 0, sizeof(input_));

  // We now setup the neural network and its parameters. The end result
  // of this operation should be that we can iterate/update all sensors in
  // a straightforward manner, likewise for the motors. We therefore first
  // create a map of all neurons, telling us how many there are for each
  // type, and what their properties are. We then iterate all sensors and
  // motors, creating the adequate neurons in place as we do so.

  // Map of ID to neuron element
  std::map<std::string, sdf::ElementPtr> neuronMap;

  // List of all hidden neurons for convenience
  std::vector<std::string> hiddenNeurons;

  // Set for tracking all collected input/output neurons
  std::set<std::string> toProcess;

  // Fetch the first neuron; note the HasElement call is necessary to prevent SDF from complaining
  // if no neurons are present.
  auto neuron = node->HasElement("rv:neuron") ? node->GetElement("rv:neuron") : sdf::ElementPtr();
  while (neuron) {
    if (not neuron->HasAttribute("layer") || not neuron->HasAttribute("id")) {
      std::cerr << "Missing required neuron attributes (id or layer). '" << std::endl;
      throw std::runtime_error("Robot brain error");
    }
    auto layer = neuron->GetAttribute("layer")->GetAsString();
    auto neuronId = neuron->GetAttribute("id")->GetAsString();

    if (layerMap_.count(neuronId)) {
      std::cerr << "Duplicate neuron ID '"
          << neuronId << "'" << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    layerMap_[neuronId] = layer;
    neuronMap[neuronId] = neuron;

    if ("input" == layer) {
      if (nInputs_ >= MAX_INPUT_NEURONS) {
        std::cerr << "The number of input neurons(" << (nInputs_ + 1)
            << ") is greater than the maximum allowed one ("
            << MAX_INPUT_NEURONS << ")" << std::endl;
        throw std::runtime_error("Robot brain error");
      }

      toProcess.insert(neuronId);
      nInputs_++;
    } else if ("output" == layer) {
      if (nOutputs_ >= MAX_OUTPUT_NEURONS) {
        std::cerr << "The number of output neurons(" << (nOutputs_ + 1)
            << ") is greater than the maximum allowed one ("
            << MAX_OUTPUT_NEURONS << ")" << std::endl;
        throw std::runtime_error("Robot brain error");
      }

      toProcess.insert(neuronId);
      nOutputs_++;
    } else if ("hidden" == layer) {
      if (hiddenNeurons.size() >= MAX_HIDDEN_NEURONS) {
        std::cerr << "The number of hidden neurons(" << (hiddenNeurons.size() + 1)
            << ") is greater than the maximum allowed one ("
            << MAX_HIDDEN_NEURONS << ")" << std::endl;
        throw std::runtime_error("Robot brain error");
      }

      hiddenNeurons.push_back(neuronId);
      nHidden_++;
    } else {
      std::cerr << "Unknown neuron layer '" << layer << "'." << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    neuron = neuron->GetNextElement("rv:neuron");
  }

  // Create motor output neurons at the correct position
  // We iterate a part's motors and just assign every
  // neuron we find in order.
  std::map<std::string, unsigned int> outputCountMap;
  unsigned int outPos = 0;
  for (auto it = motors.begin(); it != motors.end(); ++it) {
    auto motor = *it;
    auto partId = motor->partId();
    if (not outputCountMap.count(partId)) {
      outputCountMap[partId] = 0;
    }

    for (unsigned int i = 0, l = motor->outputs(); i < l; ++i) {
      std::stringstream neuronId;
      neuronId << partId << "-out-" << outputCountMap[partId];
      outputCountMap[partId]++;

      auto details = neuronMap.find(neuronId.str());
      if (details == neuronMap.end()) {
        std::cerr << "Required output neuron " << neuronId.str() <<
            " for motor could not be located"
            << std::endl;
        throw std::runtime_error("Robot brain error");
      }

      neuronHelper(&params_[outPos * MAX_NEURON_PARAMS], &types_[outPos], details->second);
      positionMap_[neuronId.str()] = outPos;
      toProcess.erase(neuronId.str());
      outPos++;
    }
  }

  // Create sensor input neurons
  std::map<std::string, unsigned int> inputCountMap;
  unsigned int inPos = 0;
  for (auto it = sensors.begin(); it != sensors.end(); ++it) {
    auto sensor = *it;
    auto partId = sensor->partId();

    if (not inputCountMap.count(partId)) {
      inputCountMap[partId] = 0;
    }

    for (unsigned int i = 0, l = sensor->inputs(); i < l; ++i) {
      std::stringstream neuronId;
      neuronId << partId << "-in-" << inputCountMap[partId];
      inputCountMap[partId]++;

      auto details = neuronMap.find(neuronId.str());
      if (details == neuronMap.end()) {
        std::cerr << "Required input neuron " << neuronId.str() <<
            " for sensor could not be located"
            << std::endl;
        throw std::runtime_error("Robot brain error");
      }

      // Input neurons can currently not have a type, so
      // there is no need to process it.
      positionMap_[neuronId.str()] = inPos;
      toProcess.erase(neuronId.str());
      inPos++;
    }
  }

  // Check if there are any input / output neurons which have not
  // yet been processed. This is an error - every input / output
  // neuron should be connected to at least a virtual motor / sensor.
  if (toProcess.size()) {
    std::cerr << "The following input / output neurons were"
        " defined, but not attached to any sensor / motor:" << std::endl;

    for (auto it = toProcess.begin(); it != toProcess.end(); ++it) {
      std::cerr << (*it) << std::endl;
    }

    std::cerr << "Create virtual sensor and motors for input / output"
        " neurons that you would like to control manually.";
    throw std::runtime_error("Robot brain error");
  }

  // Add hidden neurons
  outPos = 0;
  for (auto it = hiddenNeurons.begin(); it != hiddenNeurons.end(); ++it) {
    auto neuronId = *it;

    // Position relative to hidden neurons
    positionMap_[neuronId] = outPos;

    // Offset with output neurons within params / types
    auto pos = nOutputs_ + outPos;
    neuronHelper(&params_[pos * MAX_NEURON_PARAMS], &types_[pos], neuronMap[neuronId]);
    outPos++;
  }

  // Decode connections
  nNonInputs_ = nOutputs_ + nHidden_;
  auto connection = node->HasElement("rv:neural_connection") ? node->GetElement("rv:neural_connection") : sdf::ElementPtr();
  while (connection) {
    if (not connection->HasAttribute("src") || not connection->HasAttribute("dst")
        || not connection->HasAttribute("weight")) {
      std::cerr << "Missing required connection attributes (`src`, `dst` or `weight`)." << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    auto src = connection->GetAttribute("src")->GetAsString();
    auto dst = connection->GetAttribute("dst")->GetAsString();
    double weight;
    connection->GetAttribute("weight")->Get(weight);

    // Use connection helper to set the weight
    connectionHelper(src, dst, weight);

    // Load the next connection
    connection = connection->GetNextElement("rv:neural_connection");
  }
}

NeuralNetwork::~NeuralNetwork()
{}

void NeuralNetwork::step(double time) {
  unsigned int i = 0;
  unsigned int j = 0;

  if (nOutputs_ == 0) {
    return;
  }

  double *curState, *nextState;
  if (flipState_) {
    curState = state2_;
    nextState = state1_;
  } else {
    curState = state1_;
    nextState = state2_;
  }


  unsigned int maxNonInputs = MAX_HIDDEN_NEURONS + MAX_OUTPUT_NEURONS;
  for (i = 0; i < nNonInputs_; ++i) {
    double curNeuronActivation = 0;

    // Add input neuron values
    for (j = 0; j < nInputs_; ++j) {
      curNeuronActivation += inputWeights_[maxNonInputs * j + i]
          * input_[j];
    }

    // Add output neuron values
    for (j = 0; j < nOutputs_; ++j) {
      curNeuronActivation += outputWeights_[maxNonInputs * j + i] * curState[j];
    }

    // Add hidden neuron values
    for (j = 0; j < nHidden_; ++j) {
      curNeuronActivation += hiddenWeights_[maxNonInputs * j + i] * curState[nOutputs_ + j];
    }


    unsigned int base = MAX_NEURON_PARAMS * i;


    switch (types_[i]) {
    case SIGMOID:
      /* params are bias, gain */
      curNeuronActivation -= params_[base];
      nextState[i] = 1.0
        / (1.0 + exp(-params_[base + 1] *
            curNeuronActivation));


      break;
    case SIMPLE:
      /* linear, params are bias, gain */
      curNeuronActivation -= params_[base];
      nextState[i] = params_[base + 1] *
          curNeuronActivation;


      break;
    case OSCILLATOR: { // Use the block to prevent "crosses initialization" error
      /* params are period, phase offset, gain (amplitude) */
      double period = params_[base];
      double phaseOffset = params_[base + 1];
      double gain = params_[base + 2];

      /* Value in [0, 1] */
      nextState[i] = ((sin( (2.0*M_PI/period) *
         (time - period * phaseOffset))) + 1.0) / 2.0;

      /* set output to be in [0.5 - gain/2, 0.5 + gain/2] */
      nextState[i] = (0.5 - (gain/2.0) +
          nextState[i] * gain);

      // karine changed: add IN connections to oscillators

      nextState[i] = nextState[i] + curNeuronActivation;


    } break;
    default:
      // Unsupported type should never happen
      std::cerr << "Invalid neuron type during processing, must be a bug." << std::endl;
      throw std::runtime_error("Robot brain error");
    }
  }

  flipState_ = !flipState_;
}

void NeuralNetwork::update(const std::vector<MotorPtr>& motors,
    const std::vector<SensorPtr>& sensors,
    double t, double step) {
  boost::mutex::scoped_lock lock(networkMutex_);

  // Read sensor data and feed the neural network
  unsigned int p = 0;
  for (auto sensor : sensors) {
    sensor->read(&input_[p]);
    p += sensor->inputs();
  }

  this->step(t);

  // Since the output neurons are the first in the state
  // array we can just use it to update the motors directly.
  double * output = flipState_ ? &state2_[0] : &state1_[0];

  // Send new signals to the motors
  p = 0;
  for (auto motor: motors) {
    motor->update(&output[p], step);
    p += motor->outputs();
  }
}

//////////////////////////////////////////////////////////

void NeuralNetwork::modify(ConstModifyNeuralNetworkPtr &req) {
  boost::mutex::scoped_lock lock(networkMutex_);

  unsigned int i, j;
  for (i = 0; i < (unsigned int)req->remove_hidden_size(); ++i) {
    // Find the neuron + position
    auto id = req->remove_hidden(i);
    if (not positionMap_.count(id)) {
      std::cerr << "Unknown neuron ID `" << id << "`" << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    if ("hidden" != layerMap_[id]) {
      std::cerr << "Cannot remove neuron ID `" << id << "`, it is not a hidden neuron." << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    auto pos = positionMap_[id];
    positionMap_.erase(id);
    layerMap_.erase(id);

    // Shift types
    auto s = sizeof(types_[0]);
    std::memmove(
      // Position shifted one type to the left
      types_ + (pos + nOutputs_) * s,

      // Position of next neuron type
      types_ + (pos + nOutputs_ + 1) * s,

      // # of hidden neurons beyond this one
      s * (nHidden_ - pos - 1)
    );

    // Shift parameters
    s = sizeof(params_[0]);
    memmove(
      // Position of item to remove
      params_ + (pos + nOutputs_) * MAX_NEURON_PARAMS * s,

      // Position of next neuron type
      params_ + (pos + nOutputs_ + 1) * MAX_NEURON_PARAMS * s,

      // # of hidden neurons beyond this one
      (nHidden_ - pos - 1) * MAX_NEURON_PARAMS * s
    );

    // Reposition items in weight arrays. We start with the weights
    // of connections pointing *to* the neuron to be removed.
    // For each entry in each of the three weights arrays we have to
    // move all hidden connection weights down, then zero out the last entry
    s = sizeof(inputWeights_[0]);
    double* weightArrays[] = {inputWeights_, outputWeights_, hiddenWeights_};
    unsigned int sizes[] = {nInputs_, nOutputs_, nHidden_};

    for (unsigned int k = 0; k < 3; ++k) {
      auto * weights = weightArrays[k];
      auto size = sizes[k];

      for (j = 0; j < size; ++j) {
        memmove(
          // Position of item to remove
          weights + (nOutputs_ + pos) * s,

          // Position of next item
          weights + (nOutputs_ + pos + 1) * s,

          // # of possible hidden neurons beyond this one
          (MAX_HIDDEN_NEURONS - pos - 1) * s
        );

        // Zero out the last item in case a connection that corresponds
        // to it is ever added.
        weights[nOutputs_ + pos] = 0;
      }
    }

    // Now the weights where the removed neuron is the source
    // The block of weights corresponding to the neuron that is being
    // removed needs to be removed by shifting down all items beyond it.
    memmove(
      // Position of the item to remove
      hiddenWeights_ + pos * MAX_NON_INPUT_NEURONS * s,

      // Position of the next item
      hiddenWeights_ + (pos + 1) * MAX_NON_INPUT_NEURONS * s,

      // Remaining number of memory items
      (MAX_HIDDEN_NEURONS - pos - 1) * MAX_NON_INPUT_NEURONS * s
    );

    // Zero the remaining entries at the end
    memset(hiddenWeights_ + (MAX_HIDDEN_NEURONS - 1) * s, 0, MAX_NON_INPUT_NEURONS * s);

    // Decrement the entry in the `positionMap` for all
    // hidden neurons above this one.
    for (auto iter = positionMap_.begin(); iter != positionMap_.end(); ++iter) {
      auto layer = layerMap_[iter->first];
      if ("hidden" == layer && positionMap_[iter->first] > pos) {
        positionMap_[iter->first]--;
      }
    }

    nHidden_--;
    nNonInputs_--;
  }

  // Add new requested hidden neurons
  for (i = 0; i < (unsigned int)req->add_hidden_size(); ++i) {
    if (nHidden_ >= MAX_HIDDEN_NEURONS) {
      std::cerr << "Cannot add hidden neuron; the max ("
      << MAX_HIDDEN_NEURONS << ") is already reached." << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    auto neuron = req->add_hidden(i);
    auto id = neuron.id();
    if (layerMap_.count(id)) {
      std::cerr << "Adding duplicate neuron ID `" << id << "`" << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    positionMap_[id] = nHidden_;
    layerMap_[id] = "hidden";

    unsigned int pos = nOutputs_ + nHidden_;
    neuronHelper(&params_[pos * MAX_NEURON_PARAMS], &types_[pos], neuron);
    nHidden_++;
    nNonInputs_++;
  }

  // Update parameters of existing neurons
  for (i = 0; i < (unsigned int)req->set_parameters_size(); ++i) {
    auto neuron = req->set_parameters(i);
    auto id = neuron.id();
    if (not positionMap_.count(id)) {
      std::cerr << "Unknown neuron ID `" << id << "`" << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    auto pos = positionMap_[id];
    auto layer = layerMap_[id];

    if ("input" == layer) {
      std::cerr << "Input neurons cannot be modified." << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    neuronHelper(&params_[pos * MAX_NEURON_PARAMS], &types_[pos], neuron);
  }

  // Set weights of new or existing connections
  for (i = 0; i < (unsigned int)req->set_weights_size(); ++i) {
    auto conn = req->set_weights(i);
    auto src = conn.src();
    auto dst = conn.dst();
    this->connectionHelper(src, dst, conn.weight());
  }
}

//////////////////////////////////////

void NeuralNetwork::connectionHelper(const std::string & src, const std::string & dst, double weight) {
  if (not layerMap_.count(src)) {
    std::cerr << "Source neuron '" << src << "' is unknown." << std::endl;
    throw std::runtime_error("Robot brain error");
  }

  if (not layerMap_.count(dst)) {
    std::cerr << "Destination neuron '" << dst << "' is unknown." << std::endl;
    throw std::runtime_error("Robot brain error");
  }

  auto srcLayer = layerMap_[src];
  auto dstLayer = layerMap_[dst];

  unsigned int srcNeuronPos = positionMap_[src],
         dstNeuronPos = positionMap_[dst];

  if ("input" == dstLayer) {
    std::cerr << "Destination neuron '" << dst << "' is an input neuron." << std::endl;
    throw std::runtime_error("Robot brain error");
  } else if ("hidden" == dstLayer) {
    // Offset with output neurons for hidden neuron position
    dstNeuronPos += nOutputs_;
  }

  // Determine the index of the weight.
  unsigned int idx = (srcNeuronPos * MAX_NON_INPUT_NEURONS) + dstNeuronPos;
  if ("input" == srcLayer) {
    inputWeights_[idx] = weight;
  } else if ("output" == srcLayer) {
    outputWeights_[idx] = weight;
  } else {
    hiddenWeights_[idx] = weight;
  }
}

/////////////////////////////////////////////////

void neuronHelper(double* params, unsigned int* types, sdf::ElementPtr neuron) {
  if (not neuron->HasAttribute("type")) {
    std::cerr << "Missing required `type` attribute for neuron." << std::endl;
        throw std::runtime_error("Robot brain error");
  }

  auto type = neuron->GetAttribute("type")->GetAsString();
  if ("Sigmoid" == type || "Simple" == type) {
    types[0] = "Simple" == type ? SIMPLE : SIGMOID;

    if (not neuron->HasElement("rv:bias") || not neuron->HasElement("rv:gain")) {
      std::cerr << "A `" << type << "` neuron requires `rv:bias` and `rv:gain` elements." << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    // Set bias and gain parameters
    params[0] = neuron->GetElement("rv:bias")->Get< double >();
    params[1] = neuron->GetElement("rv:gain")->Get< double >();
  } else if ("Oscillator" == type) {
    types[0] = OSCILLATOR;

    if (not neuron->HasElement("rv:period")
        || not neuron->HasElement("rv:phase_offset")
        || not neuron->HasElement("rv:amplitude")) {
      std::cerr << "An `Oscillator` neuron requires `rv:period`"
                << ", `rv:phase_offset`"
                << ", and `rv:amplitude` elements." << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    // Set period, phase offset and gain
    params[0] = neuron->GetElement("rv:period")->Get< double >();
    params[1] = neuron->GetElement("rv:phase_offset")->Get< double >();
    params[2] = neuron->GetElement("rv:amplitude")->Get< double >();
  } else {
    std::cerr << "Unsupported neuron type `" << type << '`' << std::endl;
    throw std::runtime_error("Robot brain error");
  }
}

void neuronHelper(double* params, unsigned int* types, const revolve::msgs::Neuron & neuron) {
  auto type = neuron.type();
  if ("Sigmoid" == type || "Simple" == type) {
    types[0] = "Simple" == type ? SIMPLE : SIGMOID;
    if (neuron.param_size() != 2) {
      std::cerr << "A `" << type << "` neuron requires exactly two parameters." << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    // Set bias and gain parameters
    params[0] = neuron.param(0).value();
    params[1] = neuron.param(1).value();
  } else if ("Oscillator" == type) {
    types[0] = OSCILLATOR;

    if (neuron.param_size() != 3) {
      std::cerr << "A `" << type << "` neuron requires exactly three parameters." << std::endl;
      throw std::runtime_error("Robot brain error");
    }

    params[0] = neuron.param(0).value();
    params[1] = neuron.param(1).value();
    params[2] = neuron.param(2).value();
  } else {
    std::cerr << "Unsupported neuron type `" << type << '`' << std::endl;
    throw std::runtime_error("Robot brain error");
  }
}

} /* namespace gazebo */
} /* namespace revolve */
