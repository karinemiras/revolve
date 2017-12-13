from ..spec import BodyImplementation, NeuralNetImplementation
from ..spec.msgs import Body, BodyPart, NeuralNetwork
from ..spec.exception import err


class BodyDecoder(object):
    """
    Body decoder for the default YAML object structure
    """

    def __init__(self, spec):
        """
        :param spec:
        :type spec: BodyImplementation
        :return:
        """
        self.spec = spec
        self.part_ids = set()

    def decode(self, obj):
        """
        :param obj:
        :return:
        """
        if 'body' not in obj:
            err("Missing robot body.")

        body = Body()
        body.root.CopyFrom(self._process_body_part(obj['body']))
        return body

    def _process_body_part(self, conf, dst_slot=None):
        """
        :param conf:
        :return:
        :rtype: BodyPart
        """
        part = BodyPart()

        if 'id' not in conf:
            err("Missing part ID.")

        part.id = part_id = conf['id']
        if part_id in self.part_ids:
            err("Duplicate part ID '%s'" % part_id)
        self.part_ids.add(part_id)

        if 'type' not in conf:
            err("Missing part type.")
        part.type = part_type = conf['type']

        spec = self.spec.get(part_type)
        if spec is None:
            err("Part type '%s' not in implementation spec." % part_type)

        # Check destination slot arity
        if dst_slot is not None and dst_slot >= spec.arity:
            err("Cannot attach part '%s' with arity %d at slot %d" %
                (part_id, spec.arity, dst_slot))

        # Add part parameters
        part.orientation = conf.get('orientation', 0)

        params = spec.serialize_params(conf.get('params', {}))
        for param in params:
            p = part.param.add()
            p.value = param

        # Add children
        children = conf.get('children', {})
        for src in children:
            if src >= spec.arity:
                err("Cannot attach to slot %d of part '%s' with arity %d." %
                    (src, part_id, spec.arity))

            if src == dst_slot:
                err("Part '%s': Attempt to use slot %d for child which is already "
                    "attached to parent." % (part_id, src))
            self._process_body_connection(part, src, children[src])

        return part

    def _process_body_connection(self, part, src, conf):
        """
        :param part:
        :type part: BodyPart
        :param src: Slot on parent
        :type src: int
        :param conf:
        :return:
        :rtype: BodyConnection
        """
        conn = part.child.add()
        conn.src = src
        conn.dst = conf['slot'] if 'slot' in conf else 0
        conn.part.CopyFrom(self._process_body_part(conf, conn.dst))


class NeuralNetworkDecoder(object):
    """
    Decoder class for the standard neural network spec.
    """

    def __init__(self, spec, body_spec):
        """
        :param spec:
        :type spec: NeuralNetImplementation
        :param body_spec:
        :type body_spec: BodyImplementation
        :return:
        """
        self.spec = spec
        self.body_spec = body_spec
        self.neurons = {}

    def decode(self, obj):
        """
        :param obj:
        :return:
        :rtype: NeuralNetwork
        """
        if 'body' not in obj:
            err("Robot body required for standard Neural Network decode.")

        # Prepare all automatic input / output neurons
        #self._process_body_part(obj['body'])

        brain = obj.get('brain', {})
        neurons = brain.get('neurons', [])
        params = brain.get('params', {})
        connections = brain.get('connections', [])

       # self._create_hidden_neurons(neurons)

        # creates all nodes: input/hidden/output
        self._create_neurons(neurons,params)

        print("----------------ini-------")
        for cu in self.neurons:
            print(cu)
            for c in self.neurons[cu]:
                print(c)
                print(self.neurons[cu][c])
        print("----------------fim------")

        # Process given parameters
        #for neuron_id in params:
         #   self._process_neuron_params(neuron_id, params[neuron_id])

        nn = NeuralNetwork()
        self._process_neurons(nn)
        self._create_neuron_connections(connections, nn)

        return nn

    def _process_body_part(self, conf):
        """
        :param conf:
        :return:
        :rtype: BodyPart
        """
        part_id = conf['id']
        part_type = conf['type']

        spec = self.body_spec.get(part_type)
        if spec is None:
            err("Part type '%s' not in implementation spec." % part_type)

        # Add children
        children = conf.get('children', {})
        for src in children:
            self._process_body_part(children[src])

        # Add automatic input / output neurons
        cats = {"in": spec.inputs, "out": spec.outputs}
        for cat in cats:
            default_type = "Input" if cat == "in" else "Simple"

            for i in range(cats[cat]):
                neuron_id = "%s-%s-%d" % (part_id, cat, i)
                if neuron_id in self.neurons:
                    err("Duplicate neuron ID '%s'" % neuron_id)

                self.neurons[neuron_id] = {
                    "layer": "%sput" % cat,
                    "part_id": part_id
                }

                self._process_neuron_params(neuron_id, {"type": default_type})

    def _process_neuron_params(self, neuron_id, conf):
        """
        Processes params for a single neuron.
        :param neuron_id:
        :param conf:
        :return:
        """
        if neuron_id not in self.neurons:
            err("Cannot set parameters for unknown neuron '%s'" % neuron_id)

        current = self.neurons[neuron_id]
        if "type" not in current or "type" in conf:
            current["type"] = conf.get("type", "Simple")

        if current["type"] != "Input" and current["layer"] == "input":
            err("Input neuron '%s' must be of type 'Input'" % neuron_id)

        spec = self.spec.get(current["type"])
        if spec is None:
            err("Unknown neuron type '%s'" % current["type"])

        current["params"] = spec.serialize_params(conf)

    def _create_hidden_neurons(self, neurons):
        """
        Creates hidden neurons.
        :return:
        """
        for neuron_id in neurons:
            if neuron_id in self.neurons:
                err("Duplicate neuron ID '%s'" % neuron_id)

            # This sets the defaults, the accurate values - if present - will
            # be set by `_process_neuron_params`.
            self.neurons[neuron_id] = {
                "layer": "hidden",
                "type": "Simple"
            }

            if "part_id" in neurons[neuron_id]:
                self.neurons[neuron_id]["part_id"] = neurons[neuron_id]["part_id"]

            self._process_neuron_params(neuron_id, neurons[neuron_id])


    def _create_neurons(self, neurons, params):
        """
        Creates hidden neurons.
         #update:karinemiras
        :return:
        """
        for neuron_id in neurons:
            if neuron_id in self.neurons:
                err("Duplicate neuron ID '%s'" % neuron_id)

            # This sets the defaults, the accurate values - if present - will
            # be set by `_process_neuron_params`.
            self.neurons[neuron_id] = {
                "layer": neurons[neuron_id]['layer'],
                "type": neurons[neuron_id]['type'],
                "part_id": neurons[neuron_id]['part_id']
            }

            self.neurons[neuron_id]["params"] = []
            if(neurons[neuron_id]['layer'] != "input"):
                for p in params[neuron_id]:
                    self.neurons[neuron_id]["params"].append(params[neuron_id][p])


    def _create_neuron_connections(self, connections, brain):
        """
        Creates connections from the robot connection list.
        :param connections:
        :param brain:
        :return:
        """
        for conn in connections:
            c = brain.connection.add()
            src = conn.get("src", None)
            dst = conn.get("dst", None)
            c.weight = conn.get("weight", 0)

            if src is None:
                err("Neuron connection is missing 'src'.")

            if src not in self.neurons:
                err("Using unknown neuron '%s' as connection source." % src)

            if dst is None:
                err("Neuron connection is missing 'dst'.")

            if dst not in self.neurons:
                err("Using unknown neuron '%s' as connection destination." % dst)

            if self.neurons[dst]["layer"] == "input":
                err("Using input neuron '%s' as destination." % dst)

            c.src = src
            c.dst = dst

    def _process_neurons(self, brain):
        """
        Processes neuron data into protobuf neurons.
        :param brain:
        :type brain: NeuralNetwork
        :return:
        """
        for neuron_id in self.neurons:
            conf = self.neurons[neuron_id]
            neuron = brain.neuron.add()
            neuron.id = neuron_id
            neuron.layer = conf["layer"]
            neuron.type = conf["type"]

            if "part_id" in conf:
                neuron.partId = conf["part_id"]

            for value in conf["params"]:
                param = neuron.param.add()
                param.value = value
