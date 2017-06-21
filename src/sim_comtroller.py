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
import actionlib
import sys
import yaml
import math
import rospkg
import tf2_ros
import PyKDL as kdl
import numpy as np
from urdf_parser_py.urdf import URDF
from iai_markers_tracking.msg import MoveToGPAction, MoveToGPFeedback, MoveToGPResult
from giskard_msgs.msg import WholeBodyGoal, SemanticFloat64, ArmCommand
from kdl_parser import kdl_tree_from_urdf_model
from qpoases import PyReturnValue as returnValue
from qpoases import PySQProblem as SQProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel


class MoveToGPServer:
    _feedback = MoveToGPFeedback()
    _result = MoveToGPResult()

    def __init__(self):
        self._action_name = 'move_to_gp'
        self.server = actionlib.SimpleActionServer(self._action_name, MoveToGPAction, self.execute, False)
        self.server.start()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Variable definition
        self.grip_left = 'left_gripper_tool_frame'
        self.grip_right = 'right_gripper_tool_frame'
        self.frame_base = 'base_link'
        self.ik_lambda = 0.35
        self.slack_limit = 100
        self.eef_pose = kdl.Frame()
        # TODO: find appropriate max acceleration
        self.accel_max = np.array([0.05, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02])
        '''self.giskard = actionlib.SimpleActionClient('controller_action_server/move', WholeBodyAction)
        self.giskard.wait_for_server()
        self.pub = rospy.Publisher('/whole_body_controller/goal', WholeBodyCommand, queue_size=10)'''

        print 'Ready'

        # TODO: delete function calls from init, leave only in execute
        self.get_controller_param()
        self.get_urdf()
        self.qpoases_config()

    def execute(self, goal):
        # Variable definition
        self.goal_pose = goal.grasping_pose
        self.pose_name = self.goal_pose.child_frame_id
        self.arm = goal.arm
        self._feedback.sim_trajectory = []

        # publish info to the console for the user
        '''rospy.loginfo('%s: Executing, creating trajectory to grasping pose %s:' % (self._action_name,
                                                                                   self.pose_name))
        # rospy.loginfo(goal.grasping_pose)

        self.get_controller_param()
        self.get_urdf()
        self.qpoases_config()

        success = self.generate_trajectory()'''
        success = True
        if success:
            self._result.trajectory = self._feedback.sim_trajectory
            # rospy.loginfo('Action %s: Succeeded' % self._action_name)
            self.server.set_succeeded(self._result)

    def get_urdf(self):
        # Gets Boxy's URDF
        rospack = rospkg.RosPack()
        dir = rospack.get_path('iai_markers_tracking') + '/urdf/boxy_description.urdf'
        try:
            # self.urdf_model = urdf.Robot.from_xml_file(dir)
            self.urdf_model = URDF.from_parameter_server()
        except (OSError, LookupError) as error:
            rospy.logerr("Unexpected error while reading URDF:"), sys.exc_info()[0]

    def get_controller_param(self):
        # Read parameters from a YAML file
        try:
            pack = rospkg.RosPack()
            file_dir = pack.get_path('iai_markers_tracking') + '/config/controller_param.yaml'
            with open(file_dir, 'r') as f:
                config_file = f.read()
                self.yaml_file = yaml.load(config_file)  # Creates a dictionary
                # print self.yaml_file
        except:
            rospy.logerr("Unexpected error while reading YAML file:"), sys.exc_info()[0]
            return -1

    def generate_trajectory(self):
        # Call giskard to generate trajectory
        goal = WholeBodyGoal()
        arm = ArmCommand()
        success = False
        threshold_rot, threshold_t = SemanticFloat64(), SemanticFloat64()
        threshold_rot.value = 0.0174
        threshold_t.value = 0.001
        arm.type = ArmCommand.CARTESIAN_GOAL
        arm.goal_pose.header.frame_id = self.goal_pose.header.frame_id
        arm.goal_pose.header.stamp = rospy.get_rostime()
        arm.goal_pose.pose.position = self.goal_pose.transform.translation
        arm.goal_pose.pose.orientation = self.goal_pose.transform.rotation

        if self.arm == 'right':
            goal.command.right_ee.goal_pose = arm.goal_pose
            threshold_rot.semantics = 'r_rot_error'
            threshold_t.semantics = 'r_trans_error'
            goal.command.right_ee.convergence_thresholds.append(threshold_rot)
            goal.command.right_ee.convergence_thresholds.append(threshold_t)
        elif self.arm == 'left':
            goal.command.left_ee.goal_pose = arm.goal_pose
            threshold_rot.semantics = 'l_rot_error'
            threshold_t.semantics = 'l_trans_error'
            goal.command.left_ee.convergence_thresholds.append(threshold_rot)
            goal.command.left_ee.convergence_thresholds.append(threshold_t)
        else:
            rospy.logerr('Wrong arm specification, expected "left" or "right", received: %s'%self.arm)

        self.giskard.send_goal(goal)
        if self.giskard.wait_for_result(rospy.Duration(10)):
            state = self.giskard.get_state()
            # if state == 'SUCCEEDED':
            if state == 3:
                success = True
            rospy.loginfo('Action finished, state: %s',state)
        else:
            rospy.loginfo('Action timed out.')
            success = False

        #self.pub.publish(command)
        #rospy.loginfo(goal.command)
        print '---------------------- suc: ',success

        return success

    def qpoases_config(self):
        # QPOases needs as config parameters:
        # Nr and name of Joints - self.joint_names
        # Initial joint value   - init_joint_val
        # Links name            - links
        # Links length          - self.links_length
        # Joint weights         - self.jweights
        # Target pose           - self.goal_pose

        # Get the name of the links to simutale
        joints = self.yaml_file['simulated_joints']
        del joints[1]
        links = [self.frame_base]
        for n,joint in enumerate(joints):
            links.append(joint[0:-5]+'link')
        if self.yaml_file['arm'] == 'right':
            links.append('right_arm_7_link')
            self.gripper = self.grip_right
        else:
            links.append('left_arm_7_link')
            self.gripper = self.grip_left
        links.append(self.gripper)

        # Get links length
        self.links_length = self.get_links_length(links)

        # Joint limits: self.joint_limits_upper, self.joint_limits_lower
        self.kinem_chain(self.gripper)
        # Set initial joint vel to 0
        self.joint_velocity = [0] * self.nJoints

        # Initial joint values
        init_joint_val = []
        for key in sorted(self.yaml_file['initial_config'].iterkeys()):
            init_joint_val.append(self.yaml_file['initial_config'][key])
        init_joint_val.insert(0, init_joint_val.pop(-1))

        # Calculate initial EEF position: self.eef_pose
        self.calc_eef_position(init_joint_val)
        self.acceleration_limits()

        # Error in EEF position
        goal_orient = np.array([self.goal_pose.transform.rotation.x, self.goal_pose.transform.rotation.y,
                                self.goal_pose.transform.rotation.z, self.goal_pose.transform.rotation.w])
        eef_orient = self.rotation_to_quaternion(self.eef_pose.M)
        error_orient = goal_orient - eef_orient
        goal_posit = np.array([self.goal_pose.transform.translation.x, self.goal_pose.transform.translation.y,
                               self.goal_pose.transform.translation.z])
        eef_posit = np.array([self.eef_pose.p[0], self.eef_pose.p[1], self.eef_pose.p[2]])
        error_posit = goal_posit - eef_posit

        # Create matrix A
        A_goal = np.ones((len(error_posit)+len(error_orient), self.nJoints + 1))
        A_joint_lim = np.eye(self.nJoints + 1)
        A_joint_lim = np.delete(A_joint_lim, -1, 0)
        A_accel_lim = A_joint_lim
        A = np.concatenate((A_goal, A_joint_lim, A_accel_lim), axis=0)
        print 'A: \n', A,np.shape(A)

    def get_links_length(self, links):
        links_length = [None] * (len(links) - 1)
        for n, link in enumerate(links[0:-1]):
            try:
                distance = self.tfBuffer.lookup_transform(link, links[n + 1],
                                                               rospy.Time(0), rospy.Duration(1, 5e8))

            except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.LookupException) as exc:
                rospy.logerr('No TF found between %s and %s. ', link, links[n + 1])
                continue
            else:
                links_length[n] = math.sqrt(distance.transform.translation.x ** 2 +
                                                 distance.transform.translation.y ** 2 +
                                                 distance.transform.translation.z ** 2)
                # rospy.loginfo('Distance between %s and %s, dist %f. ', link, links[n + 1],self.dist[n])
        return links_length

    def kinem_chain(self, name_frame_end, name_frame_base='base_link'):
        # Transform URDF to Chain() for the joints between 'name_frame_end' and 'name_frame_base'
        self.chain = kdl.Chain()

        try:
            self.joint_names = self.urdf_model.get_chain(name_frame_base, name_frame_end, links=False, fixed=False)
            self.name_frame_in = name_frame_base
            self.name_frame_out = name_frame_end
            self.njoints = len(self.joint_names)

            # rospy.loginfo("Will control the following joints: %s" %(self.joint_names))

            self.kdl_tree = kdl_tree_from_urdf_model(self.urdf_model)
            self.chain = self.kdl_tree.getChain(name_frame_base, name_frame_end)
            self.kdl_fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
            self.kdl_ikv_solver = kdl.ChainIkSolverVel_wdls(self.chain)
            self.kdl_ikv_solver.setLambda(self.ik_lambda)
            # Default Task and Joint weights
            self.tweights = np.identity(6)
            # weight matrix with 1 in diagonal to make use of all the joints.
            self.jweights = np.identity(self.njoints)

            self.kdl_ikv_solver.setWeightTS(self.tweights.tolist())
            self.kdl_ikv_solver.setWeightJS(self.jweights.tolist())

            # Fill the list with the joint limits
            self.joint_limits_lower = np.empty(0)
            self.joint_limits_upper = np.empty(0)
            self.joint_vel_limits = np.empty(0)

            for jnt_name in self.joint_names:
                jnt = self.urdf_model.joint_map[jnt_name]
                if jnt.limit is not None:
                    self.joint_limits_lower = np.hstack((self.joint_limits_lower, jnt.limit.lower))
                    self.joint_limits_upper = np.hstack((self.joint_limits_upper, jnt.limit.upper))
                    self.joint_vel_limits = np.hstack((self.joint_vel_limits, jnt.limit.velocity))
            # Limits of the slack factor
            self.joint_limits_lower = np.hstack((self.joint_limits_lower, self.slack_limit))
            self.joint_limits_upper = np.hstack((self.joint_limits_upper, self.slack_limit))
            self.joint_vel_limits = np.hstack((self.joint_vel_limits, self.slack_limit))
            self.nJoints = self.chain.getNrOfJoints()

        except (RuntimeError, TypeError, NameError):
            rospy.logerr("Unexpected error:", sys.exc_info()[0])
            rospy.logerr('Could not re-init the kinematic chain')
            self.name_frame_out = ''

    def calc_eef_position(self, joint_val):
        joint_posit = kdl.JntArray(self.nJoints)
        for n,joint in enumerate(joint_posit):
            joint_posit[n] = joint_val[n]
        kinem_status = self.kdl_fk_solver.JntToCart(joint_posit, self.eef_pose)
        if kinem_status>=0:
            rospy.loginfo('Forward kinematics calculated')
        else:
            rospy.logerr('Could not calculate forward kinematics')
        return 0

    def acceleration_limits(self):
        self.ac_lim_lower = np.empty(0)
        self.ac_lim_upper = np.empty(0)
        for n,a in enumerate(self.accel_max):
            v = self.joint_vel_limits[n]
            ac_lower = ((v - a)/ v) * self.joint_velocity[n] - a
            ac_upper = ((v - a)/ v) * self.joint_velocity[n] + a
            self.ac_lim_lower = np.hstack((self.ac_lim_lower, ac_lower))
            self.ac_lim_upper = np.hstack((self.ac_lim_upper, ac_upper))
        # Limits of the slack factor
        self.ac_lim_lower = np.hstack((self.ac_lim_lower, self.slack_limit))
        self.ac_lim_upper = np.hstack((self.ac_lim_upper, self.slack_limit))

    @staticmethod
    def rotation_to_quaternion(rot_matrix):
        w = math.sqrt(1+rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2])/2
        x = (rot_matrix[2, 1] - rot_matrix[1, 2])/(4*w)
        y = (rot_matrix[0, 2] - rot_matrix[2, 0])/(4*w)
        z = (rot_matrix[1, 0] - rot_matrix[0, 1])/(4*w)
        return np.array([x, y, z, w])

if __name__ == '__main__':
    rospy.init_node('move_to_gp_server')
    rate = rospy.Rate(200)
    server = MoveToGPServer()
    rospy.spin()
