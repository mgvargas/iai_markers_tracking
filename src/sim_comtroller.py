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

import roslib
import rospy
import actionlib
import sys
import yaml
import rospkg
roslib.load_manifest('iai_markers_tracking')
from iai_markers_tracking.msg import MoveToGPAction, MoveToGPFeedback, MoveToGPResult


class MoveToGPServer:
    _feedback = MoveToGPFeedback()
    _result = MoveToGPResult()

    def __init__(self):
        self._action_name = 'move_to_gp'
        self.server = actionlib.SimpleActionServer(self._action_name, MoveToGPAction, self.execute, False)
        self.server.start()
        print 'Ready'

    def execute(self, goal):
        # Variable definition
        self.goal_pose = goal.grasping_pose
        self.pose_name = self.goal_pose.child_frame_id
        self.arm = goal.arm
        self._feedback.sim_trajectory = []

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating trajectory to grasping pose %s:' % (self._action_name,
                                                                                   self.pose_name))
        # rospy.loginfo(goal.grasping_pose)

        self.get_controller_param()

        success = self.generate_trajectory()

        if success:
            self._result.trajectory = self._feedback.sim_trajectory
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self.server.set_succeeded(self._result)

    def generate_trajectory(self):
        # Call giskard to generate trajectory

        success = True

        return success

    def get_controller_param(self):
        # Read parameters from a YAML file
        try:
            pack = rospkg.RosPack()
            file_dir = pack.get_path('iai_markers_tracking') + '/config/controller_param.yaml'
            with open(file_dir, 'r') as f:
                config_file = f.read()
                self.yaml_file = yaml.load(config_file)  # Creates a dictionary
                # print self.yaml_file
        except:  # yaml.YAMLError:
            rospy.logerr("Unexpected error while reading YAML file:"), sys.exc_info()[0]
            return -1


if __name__ == '__main__':
    rospy.init_node('move_to_gp_server')
    server = MoveToGPServer()
    rospy.spin()