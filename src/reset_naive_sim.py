#!/usr/bin/env python
# Copyright (c) 2017 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#
# Author: Minerva Gabriela Vargas Gleason <minervavargasg@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
import yaml
import rospkg
from sys import exc_info
from iai_naive_kinematics_sim.srv import SetJointState
from sensor_msgs.msg import JointState


def init_joint_state():
    # Write a YAML file with the parameters for the simulated controller
    try:
        # Open YAML configuration file
        pack = rospkg.RosPack()
        dir = pack.get_path('iai_markers_tracking') + '/config/controller_param.yaml'
        stream = open(dir, 'r')
        data = yaml.load(stream)

    except yaml.YAMLError:
        rospy.logerr("Unexpected error while reading initial configuration YAML file:"), exc_info()[0]
        return -1
    else:
        joint_w_value = data['start_config']
        joint_name = []
        joint_position = []

        for key in joint_w_value.keys():
            joint_name.append(key)
            joint_position.append(joint_w_value[key])

        effort = velocity = [0.0] * len(joint_name)
        state = JointState()
        state.name = joint_name
        state.position = joint_position
        state.velocity = velocity
        state.effort = effort

        return state


def reset_simulator():
    rospy.wait_for_service('/simulator/set_joint_states')
    try:
        init_service = rospy.ServiceProxy('/simulator/set_joint_states', SetJointState)
        rospy.loginfo('Resetting initial robot position')
        state = init_joint_state()
        init_service(state)

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == '__main__':
    try:
        reset_simulator()
    except rospy.ROSInterruptException:
        pass
