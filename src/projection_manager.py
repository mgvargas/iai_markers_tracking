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
import tf2_ros
import math
import rospkg
import sys
import yaml
import numpy as np
import PyKDL as kdl
import actionlib
from urdf_parser_py import urdf
from iai_markers_tracking.msg import Object
from iai_markers_tracking.msg import MoveToGPAction, MoveToGPGoal, MoveToGPFeedback
from iai_markers_tracking.srv import GetObjectInfo
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from kdl_parser import kdl_tree_from_urdf_model


class SelectGoal:
    def __init__(self):
        rospy.init_node('goal_selector', anonymous=True)
        rate = rospy.Rate(2)
        self.objects = rospy.Subscriber('/found_objects', Object, self.callback_obj)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.gp_action = actionlib.SimpleActionClient('move_to_gp', MoveToGPAction)

        self.grasping_poses = []
        self.object_list = []
        self.old_list = []
        self.joint_limits_lower = []
        self.joint_limits_upper = []
        self.joint_names = []
        self.gp_weights = np.zeros(6)

        # Arm selection
        self.left_arm = False
        self.right_arm = False
        self.frame_end = ''
        self.desired_chain = 'left_chain'
        self.manip_threshold = 0.05
        self.distance_threshold = 1.15
        # Gripper frames:
        self.grip_left = 'left_gripper_tool_frame'
        self.grip_right = 'right_gripper_tool_frame'
        self.frame_base = 'base_link'

        # KDL chains:
        self.right_chain = kdl.Chain()
        self.left_chain = kdl.Chain()
        self.ik_lambda = 0.35  # how much should singularities be avoided?
        self.arms_chain()  # generate chains for both arms

        self.left_jnt_pos = kdl.JntArray(self.nJoints)
        self.right_jnt_pos = kdl.JntArray(self.nJoints)
        self.jac_solver_left = kdl.ChainJntToJacSolver(self.left_chain)
        self.jac_solver_right = kdl.ChainJntToJacSolver(self.right_chain)

        self.joint_list = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        rospy.sleep(0.5)

    def callback_obj(self, objects):
        # Gets a list of the objects found by the robot (published in '/detected_objects')
        if len(objects.data) > 0:
            self.object_list = objects.data
        else:
            rospy.logerr('No objects detected by perception')
            return -1

    def joint_callback(self, data):
        # Setting the current joint angles
        self.all_joint_names = data.name
        self.joint_values = data.position
        self.odom_joints = {}
        # Getting current joint values of the arms
        a = b = 0
        for i, x in enumerate(self.all_joint_names):
            if 'left_arm' in x:
                if a < self.nJoints:
                    self.left_jnt_pos[a] = self.joint_values[i]
                    a += 1
            elif 'right_arm' in x:
                if b < self.nJoints:
                    self.right_jnt_pos[b] = self.joint_values[i]
                b += 1
            elif x == 'triangle_base_joint':
                self.triang_base_joint = {'triangle_base_joint': self.joint_values[i]}
            elif x == 'odom_x_joint':
                self.odom_joints['odom_x_joint'] = self.joint_values[i]
            elif x == 'odom_y_joint':
                self.odom_joints['odom_y_joint'] = self.joint_values[i]
            elif x == 'odom_z_joint':
                self.odom_joints['odom_z_joint'] = self.joint_values[i]

    def arms_chain(self):
        self.get_urdf()
        self.right_chain = self.kinem_chain(self.grip_right)
        self.right_joint_limits = [self.joint_limits_lower, self.joint_limits_upper]
        self.left_chain = self.kinem_chain(self.grip_left)
        self.left_joint_limits = [self.joint_limits_lower, self.joint_limits_upper]

    def get_urdf(self):
        # Gets Boxy's URDF
        # rospack = rospkg.RosPack()
        # dir = rospack.get_path('iai_markers_tracking') + '/urdf/boxy_description.urdf'
        try:
            # self.urdf_model = urdf.Robot.from_xml_file(dir)
            self.urdf_model = URDF.from_parameter_server()
        except (OSError, LookupError) as error:
            rospy.logerr('URDF not found in parameter server. ERROR',error)
            sys.exit(1)

    def init_gp_weights(self):
        # Initializes the weights of all grasping poses with 0
        if len(self.grasping_poses) > 0:
            self.gp_weights = np.zeros(len(self.grasping_poses))

    # TODO: Change to a smarter selector (from a ROS Topic maybe)
    def object_selector(self):
        # From the found objects, select one to grasp
        if self.old_list != self.object_list:
            found = False
            for obj in self.object_list:
                if obj == 'cup':
                #if obj == 'knorr_tomate':
                    self.grasping_poses_service(obj)
                    found = True
            if not found:
                pass
                # self.grasping_poses_service(self.object_list[0])
        self.old_list = self.object_list
        return self.grasping_poses

    def grasping_poses_service(self, goal_obj):
        # Calling a service to obtain the name of the grasping poses of an object
        rospy.wait_for_service('get_object_info')
        try:
            g_p = rospy.ServiceProxy('get_object_info', GetObjectInfo)
            grasping_poses_list = g_p(goal_obj)
            self.grasping_poses = grasping_poses_list.grasp_poses
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
            rospy.logerr("Service 'get_object_info' failed in obtaining the list of grasping poses")

    def goal_by_distance(self):
        # Find the grasping pose that is closer to one of the grippers
        self.trans_l = [0] * len(self.grasping_poses)
        self.trans_r = [0] * len(self.grasping_poses)
        self.dist_l = [0] * len(self.grasping_poses)
        self.dist_r = [0] * len(self.grasping_poses)
        self.pose_found = False

        for n, pose in enumerate(self.grasping_poses):
            try:
                self.trans_l[n] = self.tfBuffer.lookup_transform(self.grip_left, pose,
                                                                 rospy.Time(0), rospy.Duration(2, 5e8))
                self.trans_r[n] = self.tfBuffer.lookup_transform(self.grip_right, pose,
                                                                 rospy.Time(0), rospy.Duration(2, 5e8))
                self.pose_found = True

            except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.LookupException) as exc:
                rospy.logerr('No TF found between gripper and object. ', exc)
                continue
            else:
                self.dist_l[n] = math.sqrt(
                    self.trans_l[n].transform.translation.x ** 2 + self.trans_l[n].transform.translation.y ** 2
                    + self.trans_l[n].transform.translation.z ** 2)
                self.dist_r[n] = math.sqrt(
                    self.trans_r[n].transform.translation.x ** 2 + self.trans_r[n].transform.translation.y ** 2
                    + self.trans_r[n].transform.translation.z ** 2)

        if self.pose_found:
            self.min_dist_l = min(d for d in self.dist_l)
            self.min_dist_r = min(d for d in self.dist_r)

            # For testing, print distance to al GP
            '''print '\n Left:'
            for n in range(len(self.dist_l)):
                print '%s: %g' %(self.trans_l[n].child_frame_id, self.dist_l[n])
            print '\n Right:'
            for n in range(len(self.dist_l)):
                print '%s: %g' %(self.trans_r[n].child_frame_id, self.dist_r[n])'''

            if self.min_dist_l < self.min_dist_r:
                elem = [i for i, d in enumerate(self.dist_l) if d == self.min_dist_l]
                self.elem = elem[0]
                closest_pose = self.trans_l[self.elem]
                self.left_arm = True
                self.right_arm = False
                self.gp_weights[self.elem] += 0.4
            else:
                elem = [i for i, d in enumerate(self.dist_r) if d == self.min_dist_r]
                self.elem = elem[0]
                closest_pose = self.trans_r[self.elem]
                self.right_arm = True
                self.left_arm = False
                self.gp_weights[self.elem] += 0.4
            return closest_pose
        else:
            rospy.logerr('No TF found between gripper and object')
            return -1

    def kinem_chain(self, name_frame_end, name_frame_base='triangle_base_link'):
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
            self.joint_limits_lower = []
            self.joint_limits_upper = []
            for jnt_name in self.joint_names:
                jnt = self.urdf_model.joint_map[jnt_name]
                if jnt.limit is not None:
                    self.joint_limits_lower.append(jnt.limit.lower)
                    self.joint_limits_upper.append(jnt.limit.upper)
            self.nJoints = self.chain.getNrOfJoints()
        except:
            rospy.logerr("Unexpected error:", sys.exc_info()[0])
            rospy.logerr('Could not re-init the kinematic chain')
            self.name_frame_out = ''

        return self.chain

    def get_jacobian(self, d_chain):
        # Obtain jacobian for the selected arm
        if d_chain == 'left_chain':
            jacobian = kdl.Jacobian(self.left_chain.getNrOfJoints())
            self.jac_solver_left.JntToJac(self.left_jnt_pos, jacobian)
            # print '\n Left: \n', jacobian

        elif d_chain == 'right_chain':
            jacobian = kdl.Jacobian(self.right_chain.getNrOfJoints())
            self.jac_solver_right.JntToJac(self.right_jnt_pos, jacobian)
            # print '\n Right \n', jacobian

        else:
            rospy.logerr('Wrong chain specified for Jacobian')
            jacobian = kdl.Jacobian(self.chain.getNrOfJoints())

        return jacobian

    def arm_selector(self):
        # Find closest grasping pose of a given object
        self.object_selector()
        closest_pose = self.goal_by_distance()

        # Obtain manipulability of initial pose of arms
        left_jac = self.get_jacobian('left_chain')
        right_jac = self.get_jacobian('right_chain')
        manip_l = self.get_manipulability(left_jac)
        manip_r = self.get_manipulability(right_jac)
        # print '\n manip left: {} right: {} \n'.format(manip_l, manip_r)

        if not self.pose_found:
            rospy.logerr('No TF for given grasping_pose')
            return -1

        if self.left_arm is True and manip_l > self.manip_threshold:
            self.frame_end = self.grip_left
            self.desired_chain = 'left_chain'
            self.gp_weights[self.elem] += 0.35
        elif self.right_arm is True and manip_r > self.manip_threshold:
            self.frame_end = self.grip_right
            self.desired_chain = 'right_chain'
            self.gp_weights[self.elem] += 0.35
        else:
            dist_rate = self.min_dist_l / self.min_dist_r
            if manip_l > manip_r and dist_rate <= self.distance_threshold:
                elem = [i for i, d in enumerate(self.dist_l) if d == self.min_dist_l]
                self.elem = elem[0]
                closest_pose = self.trans_l[self.elem]
                self.gp_weights[self.elem] += 0.5
                self.frame_end = self.grip_left
                self.desired_chain = 'left_chain'
                self.left_arm = True
                self.right_arm = False
            else:
                elem = [i for i, d in enumerate(self.dist_r) if d == self.min_dist_r]
                self.elem = elem[0]
                closest_pose = self.trans_r[self.elem]
                self.gp_weights[self.elem] += 0.5
                self.frame_end = self.grip_right
                self.desired_chain = 'right_chain'
                self.left_arm = False
                self.right_arm = True

        # print '\nweights: ', self.gp_weights
        self.goal_pose = self.goal_pose_tf(closest_pose.child_frame_id)

        self.kinem_chain(self.frame_end)

        rospy.loginfo('The selected arm is {}, going to {}\n'.format(self.desired_chain,
                                                                     self.goal_pose.child_frame_id))

        return self.desired_chain

    @staticmethod
    def get_manipulability(jacobian):
        col = jacobian.columns()
        row = jacobian.rows()
        manipulability = 1

        # Arrange the jacobian as array and obtain transpose
        jac = np.zeros((row, col))
        for n in range(col):
            column = jacobian.getColumn(n)
            for m, elem in enumerate(column):
                jac[m, n] = elem
                manipulability *= elem

        jac_t = np.zeros((col, row))
        for i in range(row):
            for j in range(col):
                jac_t[j, i] = jac[i, j]

        # Manipulability = sqrt of determinant of jacobian*jacobian_transposed
        try:
            manip = math.sqrt(np.linalg.det(np.dot(jac, jac_t)))
            return manip
        except ValueError:
            rospy.logerr("No TF found, can't calculate manipulability")
            return -1

    def yaml_writer(self):
        # Write a YAML file with the parameters for the simulated controller
        try:
            # Open YAML configuration file
            pack = rospkg.RosPack()
            dir = pack.get_path('iai_markers_tracking') + '/config/controller_param.yaml'
            stream = open(dir, 'r')
            data = yaml.load(stream)

            # Get info
            sim_links_names = self.urdf_model.get_chain(self.frame_base, self.frame_end, joints=False, fixed=False)
            if self.left_arm is True:
                arm = 'left'
            else:
                arm = 'right'
            '''joint_w_values = {}
            joint_w_values.update(self.odom_joints)
            joint_w_values.update(self.triang_base_joint)
            for n, val in enumerate(self.joint_names):
                if self.left_arm is True:
                    arm = 'left'
                    joint_w_values.update({val: self.left_jnt_pos[n]})
                else:
                    arm = 'right'
                    joint_w_values.update({val: self.right_jnt_pos[n]})'''
            controlled_joint_names = self.urdf_model.get_chain('odom', self.frame_end, links=False, fixed=False)
            data['controlled_joints'] = controlled_joint_names
            data['simulated_links'] = sim_links_names
            # data['start_config'] = joint_w_values
            data['projection_mode'] = False
            data['goal_pose_name'] = self.goal_pose.child_frame_id
            data['arm'] = arm
            data['sim_frequency'] = 100
            data['watchdog_period'] = 0.1

            # Write file
            with open(dir, 'w') as outfile:
                yaml.dump(data, outfile, default_flow_style=False)
        except yaml.YAMLError:
            rospy.logerr("Unexpected error while writing controller configuration YAML file:"), sys.exc_info()[0]
            return -1

    # Possibly remove it later
    def eef_pos(self):
        # Get pose of both EEF
        l = kdl.Frame()
        r = kdl.Frame()
        fk_solver = kdl.ChainFkSolverPos_recursive(self.right_chain)
        fk_solver.JntToCart(self.right_jnt_pos, r)
        fk_solver_l = kdl.ChainFkSolverPos_recursive(self.left_chain)
        fk_solver_l.JntToCart(self.left_jnt_pos, l)
        # print '\n EEF_l: {} \n {} \n'.format(l.p, l.M)
        # print '\n EEF_r: {} \n {} \n'.format(r.p, r.M)

    def goal_pose_tf(self,pose):
        # Get TF between base and object's grasping pose
        try:
            # goal_pose = self.tfBuffer.lookup_transform(self.frame_base, pose, rospy.Time(0), rospy.Duration(1, 5e8))
            goal_pose = self.tfBuffer.lookup_transform('odom', pose, rospy.Time(0), rospy.Duration(1, 5e8))

        except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.LookupException) as exc:
            rospy.logerr('No TF found between base and object. ', exc)
            goal_pose = pose
        return goal_pose

    # TODO: Finish this function
    def dist_to_joint_limits(self, chain):
        # Obtains the distance to joint limits
        limit_warning = False
        if chain == 'left_chain':
            limit_diff_left = [0] * len(self.left_joint_limits[1])
            for n, val in enumerate(self.left_joint_limits[1]):
                limit_diff_left[n] = abs(self.left_joint_limits[1][n]) - abs(self.left_jnt_pos[n])
                if limit_diff_left[n] < 0.0001:
                    limit_warning = True
                    rospy.logwarn('left_arm_joint_{} is close to joint limits'.format(n))
            min_dist_to_limit_left = min(d for d in limit_diff_left)
        else:
            limit_diff_right = [0] * len(self.right_joint_limits[1])
            for n, val in enumerate(self.right_joint_limits[1]):
                limit_diff_right[n] = abs(self.right_joint_limits[1][n]) - abs(self.right_jnt_pos[n])
                if limit_diff_right[n] < 0.0001:
                    limit_warning = True
                    rospy.logwarn('right_arm_joint_{} is close to joint limits'.format(n))
            min_dist_to_limit_right = min(d for d in limit_diff_right)

    # TODO: Finish this action, test it
    def call_gp_action(self):
        self.gp_action.wait_for_server()
        self.feedback = MoveToGPFeedback()

        if self.left_arm:
            goal = MoveToGPGoal(grasping_pose=self.goal_pose, arm='left')
        else:
            goal = MoveToGPGoal(grasping_pose=self.goal_pose, arm='right')
        self.gp_action.send_goal(goal, feedback_cb=self.action_feedback_cb)

        # print 'Action Feedback: ',self.gp_action.feedback_cb
        state_string = self.to_string()
        state = state_string[self.gp_action.get_state()]
        rospy.loginfo('Action Initial State: {}.'.format(state))

        wait = self.gp_action.wait_for_result(rospy.Duration.from_sec(5))
        if wait:
            action_result = self.gp_action.get_result()
            state = state_string[self.gp_action.get_state()]
            rospy.loginfo('Action Result: Trajectory generated.')
            rospy.loginfo('Action state: {}.'.format(state))
            # print '------- Printing Trajectory!!\n',action_result
        else:
            self.gp_action.cancel_goal()
            self.gp_action.cancel_all_goals()
            rospy.loginfo('Action did not finish before the time out. Cancelling goal.')
            state = state_string[self.gp_action.get_state()]
            rospy.loginfo('Action state: {}. \n \n'.format(state))
            action_result = self.gp_action.get_result()
        return action_result

    def action_feedback_cb(self, msg):
        rospy.loginfo('Action Feedback: {}'.format(msg.sim_status))
        partial_trajectory = msg.sim_trajectory
        # print partial_trajectory

    @staticmethod
    def to_string():
        state = {
            0: 'PENDING',
            1: 'ACTIVE',
            2: 'PREEMPTED',
            3: 'SUCCEEDED',
            4: 'ABORTED',
            5: 'REJECTED',
            6: 'PREEMPTING',
            7: 'RECALLING',
            8: 'RECALLED',
            9: 'LOST'}
        return state

def main():
    c_goal = SelectGoal()

    while not rospy.is_shutdown():
        # Init weights
        c_goal.init_gp_weights()
        # Gets the position of the end effectors
        c_goal.eef_pos()
        # Initial arm selection based on distance to closest grasping pose and manipulability
        arm = c_goal.arm_selector()

        # Write YAML file with controller specifications
        c_goal.yaml_writer()

        # Distance to joint limits
        c_goal.dist_to_joint_limits(arm)

        c_goal.call_gp_action()
        break

    #rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
