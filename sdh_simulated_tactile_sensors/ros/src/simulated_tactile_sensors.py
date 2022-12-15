#!/usr/bin/env python3
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import rospy
import numpy as np
from schunk_sdh.msg import TactileMatrix, TactileSensor
from gazebo_msgs.msg import ContactsState, LinkStates
from schunk_sdh_ros.msg import ContactInfo, ContactInfoArray
from tf.transformations import quaternion_matrix, quaternion_inverse


class GazeboTactilePad(object):
    def __init__(self, topic_name, frame_name, finger_length, finger_width):
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.state_callback)

        self.finger_length = finger_length
        self.finger_width = finger_width

        self.texel_width = rospy.get_param("~texel_width", 3.4)
        self.texel_height = rospy.get_param("~texel_height", 3.4)
        self.texel_area = self.texel_width * self.texel_height
        self.contact_force_cell_threshold = rospy.get_param("~contact_force_cell_threshold", 0.001)

        self.sensitivity = rospy.get_param("~sensitivity", 20.0)
        rospy.logdebug("sensitivity: %f", self.sensitivity)

        # get parameters from parameter server
        self.cells_x = rospy.get_param("~cells_x", 6)
        self.cells_y = rospy.get_param("~cells_y", 14)
        rospy.logdebug("size of the tactile matrix is %ix%i patches", self.cells_x, self.cells_y)

        self.range = rospy.get_param("~output_range", 3500)
        rospy.logdebug("output range: %f", self.range)

        self.filter_length = rospy.get_param("~filter_length", 5)
        rospy.logdebug("filter length: %i", self.filter_length)

        self.smoothed_matrix = self.create_empty_force_matrix()

        rospy.wait_for_message("/gazebo/link_states", LinkStates)

        self.frame_name = frame_name
        self.frame_idx = self.link_state.name.index(self.frame_name)

        rospy.Subscriber(topic_name, ContactsState, self.contact_callback)
        rospy.logdebug("subscribed to bumper states topic: %s", topic_name)

        rospy.wait_for_message(topic_name, ContactsState)

    def get_transformation(self):
        pose = self.link_state.pose[self.frame_idx]

        q = quaternion_inverse([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        self.R = quaternion_matrix(q)
        self.T = np.ones((4, 1))
        self.T[0] = pose.position.x
        self.T[1] = pose.position.y
        self.T[2] = pose.position.z

    def transform_to_local_frame(self, global_vec):
        r = np.ones((4, 1))
        r[0] = global_vec.x - self.T[0]
        r[1] = global_vec.y - self.T[1]
        r[2] = global_vec.z - self.T[2]
        local_vec = self.R.dot(r)
        return local_vec

    def state_callback(self, msg):
        self.link_state = msg

    def contact_callback(self, states):
        self.states = states

    def get_smoothed_matrix(self):
        matrix = self.create_empty_force_matrix()
        matrix = self.update(self.states, matrix)
        matrix = self.smooth(matrix)
        matrix = self.time_average(self.smoothed_matrix, matrix, self.filter_length)
        self.smoothed_matrix = matrix

    def update(self, states, matrix):
        """
        Update the provided matrix with the contact information from the Gazebo
        simulator.

        :param states: gazebo_msgs.msg.ContactsState
        :param matrix: Float[]
        """
        # no contact, so we don't update the matrix
        if len(states.states) == 0:
            return matrix
        self.get_transformation()
        for i in range(len(states.states)):
            state = states.states[i]
            for j in range(len(state.contact_positions)):
                # check if the contact is on the same side as the sensor
                contact_position = self.transform_to_local_frame(state.contact_positions[j])

                # normalize the contact position along the tactile sensor
                # we assume that the tactile sensor occupies the complete
                # surface of the inner finger side, so finger size is equal to
                # sensor size
                normalized_x = contact_position[1] / self.finger_width + 0.5
                normalized_y = contact_position[2] / self.finger_length
                # from the normalized coordinate we can now determine the index
                # of the tactile patch that is activated by the contact
                x = round(normalized_x * self.cells_x)
                y = round(normalized_y * self.cells_y)
                force = abs(state.wrenches[j].force.x)

                index = int(self.get_index(x, y))

                matrix[index] += force

        return matrix

    def create_empty_force_matrix(self):
        """
        Create a new matrix that contains the current force on each cell. Initialize
        all cells with zeros.

        :return: Float[]
        """
        matrix = []
        for i in range(self.cells_x * self.cells_y):
            matrix.append(0.0)

        return matrix

    def smooth(self, matrix):
        """
        Run a moving average on the provided matrix.

        :param matrix: Float[]
        :return: Float[]
        """
        smoothed_matrix = self.create_empty_force_matrix()

        for x in range(0, self.cells_x):
            for y in range(0, self.cells_y):
                sum = 0.0
                count = 0
                for dx in range(-1, 2):
                    index_x = x + dx
                    if index_x < 0 or index_x >= self.cells_x:
                        continue

                    for dy in range(-1, 2):
                        index_y = y + dy
                        if index_y < 0 or index_y >= self.cells_y:
                            continue

                        index = self.get_index(index_x, index_y)
                        sum += matrix[index]
                        count += 1
                index = self.get_index(x, y)
                smoothed_matrix[index] = sum / count

        return smoothed_matrix

    def time_average(self, matrix_buffer, current_matrix, filter_length):
        """
        Calculate the average matrix from the force buffer.

        :return: Float[]
        """
        matrix = self.create_empty_force_matrix()
        sample_factor = 1.0 / filter_length

        for i in range(self.cells_x * self.cells_y):
            force = (1.0 - sample_factor) * matrix_buffer[i]
            force += sample_factor * current_matrix[i]
            matrix[i] = force

        return matrix

    def tactile_matrix(self, matrix_id):
        """
        Get the current forces as tactile matrix. matrix_id is the identifier of the
        tactile matrix and determines which pad produced the data.

        :param matrix_id: Integer
        :return: schunk_sdh.msg.TactileMatrix
        """
        matrix = TactileMatrix()
        matrix.matrix_id = matrix_id
        matrix.cells_x = self.cells_x
        matrix.cells_y = self.cells_y

        m = self.smoothed_matrix

        for i in range(self.cells_x * self.cells_y):
            force = m[i] * self.sensitivity
            if force < 0.0:
                force = 0.0
            if force > self.range:
                force = self.range
            matrix.tactile_array.append(int(force))

        return matrix

    def contact_matrix(self, matrix_id):
        matrix = ContactInfo()
        matrix.matrix_id = matrix_id

        m = self.smoothed_matrix
        cf = self.GetContactForce(m)
        matrix.force = cf[0]
        if matrix.force > 0:
            matrix.in_contact = True
        else:
            matrix.in_contact = False
        matrix.x_center = cf[1]
        matrix.y_center = cf[2]
        matrix.contact_area = cf[3]

        return matrix

    def get_index(self, x, y):
        """
        Map the two-dimensional coordinate of a tactile patch to an index in the
        one-dimensional data array. The coordinates are bound to the upper and lower
        limits.

        :param x: Integer
        :param y: Integer
        :return: Integer
        """
        y = self.cells_y - y - 1
        if x >= self.cells_x:
            x = self.cells_x - 1
        if x < 0:
            x = 0
        if y >= self.cells_y:
            y = self.cells_y - 1
        if y < 0:
            y = 0

        return y * self.cells_x + x

    def GetContactForce(self, m):
        """
        Return a tuple (force,cog_x,cog_y,area) of contact force and
        center of gravity and contact area of that force
        force is in N, cog_x,cog_ in mm, area in mm*mm.
        """
        sum_forces = 0.0
        sum_x = 0.0
        sum_y = 0.0
        nbcells = 0

        i = 0
        for y in range(self.cells_y):
            for x in range(self.cells_x):
                force = m[i]
                if force < 0.0:
                    force = 0.0
                if force > self.contact_force_cell_threshold:
                    if force > self.range:
                        force = self.range
                    f = force
                    sum_forces += f
                    sum_x += float(x) * f
                    sum_y += float(y) * f
                    nbcells += 1
                i += 1

        area = self.texel_area * float(nbcells)

        force = sum_forces
        if sum_forces > 0.001:
            cog_x = self.texel_width * sum_x / sum_forces
            cog_y = self.texel_height * sum_y / sum_forces
        else:
            cog_x = 0.0
            cog_y = 0.0

        return (force, cog_x, cog_y, area)


class GazeboVirtualTactileSensor(object):
    """
    Constants that determine which indices the finger parts have in the
    schunk_sdh.msg.TactileSensor matrix.
    """

    ID_THUMB_2 = 0
    ID_THUMB_3 = 1
    ID_FINGER_12 = 2
    ID_FINGER_13 = 3
    ID_FINGER_22 = 4
    ID_FINGER_23 = 5

    def __init__(self):
        self.pads = []
        self.pads.append(GazeboTactilePad("/sdh_thumb_2_bumper", "sdh::sdh_thumb_2_link", 0.0865, 0.03))
        self.pads.append(GazeboTactilePad("/sdh_thumb_3_bumper", "sdh::sdh_thumb_3_link", 0.0675, 0.03))
        self.pads.append(GazeboTactilePad("/sdh_finger_12_bumper", "sdh::sdh_finger_12_link", 0.0865, 0.03))
        self.pads.append(GazeboTactilePad("/sdh_finger_13_bumper", "sdh::sdh_finger_13_link", 0.0675, 0.03))
        self.pads.append(GazeboTactilePad("/sdh_finger_22_bumper", "sdh::sdh_finger_22_link", 0.0865, 0.03))
        self.pads.append(GazeboTactilePad("/sdh_finger_23_bumper", "sdh::sdh_finger_23_link", 0.0675, 0.03))

        self.pub_tactile_data = rospy.Publisher("~tactile_data", TactileSensor, queue_size=1)
        rospy.loginfo("'tactile_data' topic advertized")

        self.pub_contact_info = rospy.Publisher("~contact_info", ContactInfoArray, queue_size=1)
        rospy.loginfo("'contact_info' topic advertized")

    def publish_data(self):
        """
        Publish the current state of the simulated tactile sensors.
        """
        tactile_msg = TactileSensor()
        contact_msg = ContactInfoArray()
        tactile_msg.header.stamp = contact_msg.header.stamp = rospy.Time.now()
        for i in range(6):
            self.pads[i].get_smoothed_matrix()
            tactile_msg.tactile_matrix.append(self.pads[i].tactile_matrix(i))
            contact_msg.contact_info.append(self.pads[i].contact_matrix(i))
        self.pub_tactile_data.publish(tactile_msg)
        self.pub_contact_info.publish(contact_msg)


if __name__ == "__main__":
    rospy.init_node("tactile_sensors")
    rospy.sleep(0.5)
    rate = rospy.get_param("~rate", 90)
    r = rospy.Rate(rate)
    sensor = GazeboVirtualTactileSensor()

    while not rospy.is_shutdown():
        sensor.publish_data()
        r.sleep()
