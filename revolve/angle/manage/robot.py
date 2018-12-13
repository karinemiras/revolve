from collections import deque

from sdfbuilder.math import Vector3, Quaternion
from revolve.util import Time
import numpy as np
import math

class Robot(object):
    """
    Class to manage a single robot with the WorldManager
    """

    def __init__(self, name, tree, robot, position, time, battery_level=0.0,
                 speed_window=60, warmup_time=0, parents=None):
        """
        :param speed_window:
        :param name:
        :param tree:
        :param robot: Protobuf robot
        :param position:
        :type position: Vector3
        :param time:
        :type time: Time
        :param battery_level:
        :type battery_level: float
        :param parents:
        :type parents: set
        :return:
        """
        self.warmup_time = warmup_time
        self.speed_window = speed_window
        self.tree = tree
        self.robot = robot
        self.name = name
        self.starting_position = position
        self.starting_time = time
        self.battery_level = battery_level

        self.last_position = position
        self.last_update = time
        self.last_mate = None

        self.parent_ids = set() if parents is None else set(p.robot.id for p in parents)
        self._ds = deque(maxlen=speed_window)
        self._dt = deque(maxlen=speed_window)
        self._positions = deque(maxlen=speed_window)
        self._orientations = deque(maxlen=speed_window)
        self._seconds =  deque(maxlen=speed_window)
        self._times = deque(maxlen=speed_window)

        self._dist = 0
        self._time = 0
        self._idx = 0
        self._count = 0

        self.second = 1
        self.count_group = 1
        self.avg_roll = 0
        self.avg_pitch = 0
        self.avg_yaw = 0
        self.avg_x = 0
        self.avg_y = 0
        self.avg_z = 0


    def write_robot(self, world, details_file, csv_writer):
        """
        Writes this robot to a file. This simply writes the
        protobuf bot to a file, which can later be recovered

        :param world: The world
        :param details_file:
        :param csv_writer:
        :type csv_writer: csv.writer
        :return:
        :rtype: bool
        """
        with open(details_file, 'w') as f:
            f.write(self.robot.SerializeToString())

        row = [self.robot.id]
        row += list(self.parent_ids) if self.parent_ids else ['', '']
        row += [self.get_battery_level()]
        csv_writer.writerow(row)

    def update_state(self, world, time, state, poses_file):
        """
        Updates the robot state from a state message.

        :param world: Instance of the world
        :param time: The simulation time at the time of this
                     position update.
        :type time: Time
        :param state: State message
        :param poses_file: CSV writer to write pose to, if applicable
        :type poses_file: csv.writer
        :return:
        """
        pos = state.pose.position
        position = Vector3(pos.x, pos.y, pos.z)

        rot = state.pose.orientation
        qua = Quaternion(rot.w, rot.x, rot.y, rot.z)
        euler = qua.get_rpy()
        euler = np.array([euler[0],euler[1],euler[2]]) # roll / pitch / yaw

        age = world.age()

        if self.starting_time is None:
            self.starting_time = time
            self.last_update = time
            self.last_position = position

        if poses_file:
            age = world.age()
            poses_file.writerow([self.robot.id, age.sec, age.nsec,
                                 position.x, position.y, position.z,
                                 self.get_battery_level()])

        if float(self.age()) < self.warmup_time:
            # Don't update position values within the warmup time
            self.last_position = position
            self.last_update = time
            return

        # Calculate the distance the robot has covered as the Euclidean distance over
        # the x and y coordinates (we don't care for flying), as well as the time
        # it took to cover this distance.
        last = self.last_position
        ds = np.sqrt((position.x - last.x)**2 + (position.y - last.y)**2)
        dt = float(time - self.last_update)

        # Velocity is of course sum(distance) / sum(time)
        # Storing all separate distance and time values allows us to
        # efficiently calculate the new speed over the window without
        # having to sum the entire arrays each time, by subtracting
        # the values we're about to remove from the _dist / _time values.
        self._dist += ds
        self._time += dt

        if len(self._dt) >= self.speed_window:
            # Subtract oldest values if we're about to override it
            self._dist -= self._ds[-1]
            self._time -= self._dt[-1]

        self.last_position = position
        self.last_update = time

        self._positions.append(position)
        self._times.append(time)
        self._ds.append(ds)
        self._dt.append(dt)
        self._orientations.append(euler)
        self._seconds.append(age.sec)


    def velocity(self):
        """
        Returns the velocity over the maintained window
        :return:
        """
        return self._dist / self._time if self._time > 0 else 0

    def displacement(self):
        """
        Returns a tuple of the displacement in both time and space
        between the first and last registered element in the speed
        window.
        :return: Tuple where the first item is a displacement vector
                 and the second a `Time` instance.
        :rtype: tuple(Vector3, Time)
        """
        if self.last_position is None:
            return Vector3(0, 0, 0), Time()

        return (
            self._positions[-1] - self._positions[0],
            self._times[-1] - self._times[0]
        )


    def head_balance(self):

        roll = 0
        pitch = 0

        it = len(self._orientations)

        for o in self._orientations:

            roll = roll + abs(o[0])* 180 / math.pi
            pitch = pitch + abs(o[1])* 180 / math.pi

        #  accumulated angles for each type of rotation
        #  divided by iterations * maximum angle * each type of rotation
        balance = (roll + pitch) / (it * 180 * 2)

        balance = 1 - balance # imbalance to balance




    def export_positions(self, evaluation_time, robotid, generation, experiment_name):


        it = len(self._seconds)

        for o in range(0, it):

            if o < it-1:
                if self._seconds[o] != self._seconds[o+1]:
                    self.write_pos(o, evaluation_time, robotid, generation, experiment_name)
                else:
                    self.accumulates_pos(o)
            else:
                if self._seconds[o] != self._seconds[o-1]:
                    self.write_pos(o, evaluation_time, robotid, generation, experiment_name)
                else:
                    self.accumulates_pos(o)


    def write_pos(self, o, evaluation_time, robotid, generation, experiment_name):

        f = open('../../../l-system/experiments/'+ experiment_name+'/offspringpop'+generation+'/positions_'+robotid+'.txt', "a+")

        if self.second<= evaluation_time:

            self.avg_roll += self._orientations[o][0]
            self.avg_pitch += self._orientations[o][1]
            self.avg_yaw += self._orientations[o][2]
            self.avg_x += self._positions[o].x
            self.avg_y += self._positions[o].y
            self.avg_z += self._positions[o].z

            self.avg_roll = self.avg_roll/self.count_group
            self.avg_pitch = self.avg_pitch/self.count_group
            self.avg_yaw = self.avg_yaw/self.count_group
            self.avg_x = self.avg_x/self.count_group
            self.avg_y = self.avg_y/self.count_group
            self.avg_z = self.avg_z/self.count_group

            if self.avg_roll < 0:
                self.avg_roll =   360 + (self.avg_roll * 180 / math.pi )
            else:
                self.avg_roll = self.avg_roll * 180 / math.pi

            if self.avg_pitch < 0:
                self.avg_pitch =  360 + (self.avg_pitch * 180 / math.pi)
            else:
                self.avg_pitch = self.avg_pitch * 180 / math.pi

            if self.avg_yaw < 0:
                self.avg_yaw =    360 + (self.avg_yaw* 180 / math.pi)
            else:
                self.avg_yaw =    self.avg_yaw* 180 / math.pi

            f.write(str(self.second) + ' ' + str(self.avg_roll) + ' ' + str(self.avg_pitch) + ' ' + str(self.avg_yaw) + ' ' + str(self.avg_x) + ' ' + str(self.avg_y) + ' ' + str(self.avg_z) + '\n')

            self.second += 1
            self.avg_roll = 0
            self.avg_pitch = 0
            self.avg_yaw = 0
            self.avg_x = 0
            self.avg_y = 0
            self.avg_z = 0
            self.count_group = 1

        f.close()

    def accumulates_pos(self, o):

        self.avg_roll += self._orientations[o][0]
        self.avg_pitch += self._orientations[o][1]
        self.avg_yaw += self._orientations[o][2]
        self.avg_x += self._positions[o].x
        self.avg_y += self._positions[o].y
        self.avg_z += self._positions[o].z
        self.count_group += 1


    def displacement_velocity(self):
        """
        Returns the displacement velocity, i.e. the velocity
        between the first and last recorded position of the
        robot in the speed window over a straight line,
        ignoring the path that was taken.
        :return:
        """
        dist, time = self.displacement()
        if time.is_zero():
            return 0.0

        return np.sqrt(dist.x**2 + dist.y**2) / float(time)

    def age(self):
        """
        Returns this robot's age as a Time object.
        Depends on the last and first update times.
        :return:
        :rtype: Time
        """
        return Time() if self.last_update is None else self.last_update - self.starting_time

    def get_battery_level(self):
        """
        Method to return the robot battery level. How the battery level
        is managed is probably implementation specific, so you'll likely
        have to modify this method for your specific use.
        :return:
        """
        return self.battery_level
