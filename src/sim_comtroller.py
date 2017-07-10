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
from giskard_msgs.msg import WholeBodyGoal, WholeBodyCommand, SemanticFloat64, ArmCommand, WholeBodyAction
from iai_naive_kinematics_sim.msg import ProjectionClock
from kdl_parser import kdl_tree_from_urdf_model
from sensor_msgs.msg import JointState
from qpoases import PyReturnValue as returnValue
from qpoases import PySQProblem as SQProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel


class MoveToGPServer:
    _feedback = MoveToGPFeedback()
    _result = MoveToGPResult()

    def __init__(self):
        self._action_name = 'move_to_gp'
        self.server = actionlib.SimpleActionServer(self._action_name, MoveToGPAction, self.execute_action, False)
        self.server.start()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # self.giskard = actionlib.SimpleActionClient('controller_action_server/move', WholeBodyAction)

        # Variable definition
        self.grip_left = 'left_gripper_tool_frame'
        self.grip_right = 'right_gripper_tool_frame'
        self.frame_base = 'base_link'
        self.ik_lambda = 0.35
        self.slack_limit = 300
        self.nJoints = 8+3
        self.arm = 'left'
        self.joint_values = np.empty([self.nJoints])
        self.joint_values_kdl = kdl.JntArray(self.nJoints)
        self.eef_pose = kdl.Frame()
        # TODO: find appropriate max acceleration
        self.accel_max = np.array([0.1, 0.1, 0.1, 0.05, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02])

        rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.pub_clock = rospy.Publisher('/simulator/projection_clock', ProjectionClock, queue_size=3)
        self.pub_velocity = rospy.Publisher('/simulator/commands', JointState, queue_size=10)

        print 'Ready'

    def joint_callback(self, data):
        # Getting current joint values of the arms
        self.all_joint_names = data.name
        self.all_joint_values = data.position
        self.start_arm = [0]*(self.nJoints-1)
        self.start_odom = [0]*3
        a, n = 4, 0#1
        if self.arm == 'right':
            arm = 'right_arm'
            self.gripper = self.grip_right
        else:
            arm = 'left_arm'
            self.gripper = self.grip_left

        for i, x in enumerate(self.all_joint_names):
            if arm in x and (a-3) < (self.nJoints):
                try:
                    self.joint_values[a] = self.all_joint_values[i]
                    self.joint_values_kdl[a] = self.all_joint_values[i]
                    self.start_arm[a-4] = i
                    a += 1
                except IndexError:
                    index_error = True
            elif 'triangle_base' in x:
                try:
                    self.joint_values[3] = self.all_joint_values[i]
                    self.joint_values_kdl[3] = self.all_joint_values[i]
                    self.triang_list_pos = i
                except IndexError:
                    index_error = True
            elif 'odom' in x:
                try:
                    self.joint_values[n] = self.all_joint_values[i]
                    self.joint_values_kdl[n] = self.all_joint_values[i]
                    self.start_odom[n] = i
                    n += 1
                except IndexError:
                    index_error = True

        try:
            self.calc_eef_position(self.joint_values)
        except AttributeError:
            pass

    def execute_action(self, goal):
        # Variable definition
        self.goal_pose = goal.grasping_pose
        self.pose_name = self.goal_pose.child_frame_id
        self.arm = goal.arm
        self._feedback.sim_trajectory = []

        # publish info to the console for the user
        rospy.loginfo('%s: Executing, creating trajectory to grasping pose %s:' % (self._action_name,
                                                                                   self.pose_name))
        # rospy.loginfo(goal.grasping_pose)
        np.set_printoptions(suppress=True, precision=2, linewidth=120)
        self.get_urdf()
        self.qpoases_config()
        self.qpoases_calculation()

        # success = self.generate_trajectory()
        success = True
        if success:
            self._result.trajectory = self._feedback.sim_trajectory
            rospy.loginfo('Action %s: Succeeded' % self._action_name)
            self.server.set_succeeded(self._result)

    def get_urdf(self):
        # Gets Boxy's URDF
        # rospack = rospkg.RosPack()
        # dir = rospack.get_path('iai_markers_tracking') + '/urdf/boxy_description.urdf'
        try:
            # self.urdf_model = urdf.Robot.from_xml_file(dir)
            self.urdf_model = URDF.from_parameter_server()
        except (OSError, LookupError) as error:
            rospy.logerr("Unexpected error while reading URDF:"), sys.exc_info()[0]

    # TODO: delete this function
    def generate_trajectory(self):
        # Call giskard to generate trajectory
        self.giskard.wait_for_server()
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
            goal.command.right_ee = arm
            threshold_rot.semantics = 'r_rot_error'
            threshold_t.semantics = 'r_trans_error'
            goal.command.right_ee.convergence_thresholds.append(threshold_rot)
            goal.command.right_ee.convergence_thresholds.append(threshold_t)
        elif self.arm == 'left':
            goal.command.left_ee = arm
            threshold_rot.semantics = 'l_rot_error'
            threshold_t.semantics = 'l_trans_error'
            goal.command.left_ee.convergence_thresholds.append(threshold_rot)
            goal.command.left_ee.convergence_thresholds.append(threshold_t)
        else:
            rospy.logerr('Wrong arm specification, expected "left" or "right", received: %s'%self.arm)

        self.giskard.send_goal(goal)
        if self.giskard.wait_for_result(rospy.Duration(10)):
            state = self.giskard.get_state()
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
        np.set_printoptions(suppress=True, precision=2, linewidth=120)

        #Variables
        n_slack = 6
        self.sweights = np.ones((n_slack))*2

        # Joint limits: self.joint_limits_upper, self.joint_limits_lower
        self.kinem_chain(self.gripper)
        # Set initial joint vel to 0
        self.joint_velocity = [0] * self.nJoints

        # Calculate acceleration limits
        self.acceleration_limits()

        # Error in EEF position
        self.goal_quat = np.array([self.goal_pose.transform.rotation.x, self.goal_pose.transform.rotation.y,
                         self.goal_pose.transform.rotation.z, self.goal_pose.transform.rotation.w])
        # self.goal_orient = self.quaternion_to_euler(self.goal_quat)
        eef_orient_quat = self.rotation_to_quaternion(self.eef_pose.M)
        q_interp = self.slerp(eef_orient_quat, self.goal_quat, t=0.1)
        # eef_orient = self.quaternion_to_euler(eef_orient_quat)
        # self.error_orient = self.goal_orient - eef_orient
        self.error_orient = self.quaternion_to_euler(q_interp)

        self.goal_posit = np.array([self.goal_pose.transform.translation.x, self.goal_pose.transform.translation.y,
                               self.goal_pose.transform.translation.z])
        eef_posit = np.array([self.eef_pose.p[0], self.eef_pose.p[1], self.eef_pose.p[2]])
        self.error_posit = self.goal_posit - eef_posit

        # Get jacobian
        jacobian = self.get_jacobian()
        # print '\n Jacobian: \n', jacobian

        # Create matrix A, one slack factor per DOF of the EEF
        A_goal = np.hstack((jacobian, np.eye(n_slack)))
        A_joint_lim = np.hstack((np.eye(self.nJoints), np.zeros((self.nJoints, n_slack))))
        A_accel_lim = A_joint_lim
        # self.A = np.concatenate((A_goal, A_joint_lim, A_accel_lim), axis=0)
        self.A = np.concatenate((A_goal, A_joint_lim), axis=0)
        self.problem_size = np.shape(self.A)
        # print 'A ', np.shape(self.A), '\n',self.A

        # Boundary vectors of A
        self.joint_dist_lower_lim = np.empty([self.nJoints])
        self.joint_dist_upper_lim = np.empty([self.nJoints])
        self.joint_dist_lower_lim = np.subtract(self.joint_limits_lower, self.joint_values)
        self.joint_dist_upper_lim = np.subtract(self.joint_limits_upper, self.joint_values)
        # self.lbA = np.hstack((self.error_posit, self.error_orient, self.joint_dist_lower_lim, self.ac_lim_lower))
        # self.ubA = np.hstack((self.error_posit, self.error_orient, self.joint_dist_upper_lim, self.ac_lim_upper))
        self.lbA = np.hstack((self.error_posit, self.error_orient, self.joint_dist_lower_lim))
        self.ubA = np.hstack((self.error_posit, self.error_orient, self.joint_dist_upper_lim))
        # print 'lbA ', np.shape(self.lbA), '\n', self.lbA
        # print 'ubA ', np.shape(self.ubA), '\n', self.ubA

        # Create vector g
        self.g = np.zeros(self.nJoints+n_slack)
        # print 'g ', np.shape(self.g), '\n', self.g

        # Boundary vectors of the state vector
        slack_vel = np.ones((n_slack))*self.slack_limit
        self.lb = np.hstack((-self.joint_vel_limits, -slack_vel))
        self.ub = np.hstack((self.joint_vel_limits, slack_vel))
        # print 'lb ', np.shape(self.lb), '\n', self.lb
        # print 'ub ', np.shape(self.ub), '\n', self.ub

        # Define matrix H
        self.H = np.diag(np.hstack((np.diag(self.jweights), self.sweights)))
        # print 'self.H ', np.shape(self.H), '\n',self.H

    def qpoases_calculation(self):
        # Variable initialization
        p = 10
        nWSR = np.array([100])
        limit_o = abs(self.error_orient)
        limit_p = abs(self.error_posit)

        vel_calculation = SQProblem(self.problem_size[1], self.problem_size[0])
        print (self.problem_size[1], self.problem_size[0])

        # Setting up QProblem object
        options = Options()
        options.setToReliable()
        options.printLevel = PrintLevel.LOW
        vel_calculation.setOptions(options)
        Opt = np.zeros(self.problem_size[1])
        i = 0
        precision = [0.1, 0.1, 0.03]

        return_value = vel_calculation.init(self.H, self.g, self.A, self.lb, self.ub, self.lbA, self.ubA, nWSR)

        if return_value != returnValue.SUCCESSFUL_RETURN:
            rospy.logerr("Init of QP-Problem returned without success!")
            return -1

        vel_calculation.getPrimalSolution(Opt)

        # Config iai_naive_kinematics_sim, send commands
        clock = ProjectionClock()
        velocity_msg = JointState()
        velocity_msg.name = self.all_joint_names
        velocity_msg.header.stamp = rospy.get_rostime()
        velocity_array = [0.0 for x in range(len(self.all_joint_names))]
        effort_array = [0.0 for x in range(len(self.all_joint_names))]
        velocity_msg.velocity = velocity_array
        velocity_msg.effort = effort_array
        velocity_msg.position = effort_array

        clock.now = rospy.get_rostime()
        clock.period.nsecs = 10000000
        self.pub_clock.publish(clock)
        print 'error: ', self.error_posit

        while np.any(limit_p > precision) and i < 600:
            # Solve QP.
            i += 1
            nWSR = np.array([100])
            self.acceleration_limits()
            self.joint_dist_lower_lim = self.joint_limits_lower - self.joint_values
            self.joint_dist_upper_lim = self.joint_limits_upper - self.joint_values
            self.lbA = np.hstack((self.error_posit, self.error_orient, self.joint_dist_lower_lim))
            self.ubA = np.hstack((self.error_posit, self.error_orient, self.joint_dist_upper_lim))

            # recalculate H matrix
            self.calculate_weigths()

            # return_value = vel_calculation.hotstart(self.H, self.g, self.A, self.lb, self.ub, self.lbA, self.ubA, nWSR)
            return_value = vel_calculation.init(self.H, self.g, self.A, self.lb, self.ub, self.lbA, self.ubA, nWSR)

            if return_value != returnValue.SUCCESSFUL_RETURN:
                rospy.logerr("Hotstart of QP-Problem returned without success! ERROR MESSAGE: ")
                return -1

            for x,vel in enumerate(Opt[4:(self.nJoints)]):
                p = self.start_arm[x]
                velocity_msg.velocity[p] = vel
            # If error and velocity have different signs, correct it
            if self.error_posit[2] * Opt[3] < 0:
                velocity_msg.velocity[self.triang_list_pos] = -Opt[3]
            else:
                velocity_msg.velocity[self.triang_list_pos] = Opt[3]
            if self.error_posit[0]*Opt[0] < 0:
                velocity_msg.velocity[0] = -Opt[0]
            else:
                velocity_msg.velocity[0] = Opt[0]
            if self.error_posit[1]*Opt[1] < 0:
                velocity_msg.velocity[1] = -Opt[1]
            else:
                velocity_msg.velocity[1] = Opt[1]

            # velocity_msg.position = self.all_joint_values
            # print velocity_msg
            for n,j in enumerate(velocity_msg.name):
                print j, velocity_msg.velocity[n]

            # Recalculate Error in EEF position
            eef_orient_quat = self.rotation_to_quaternion(self.eef_pose.M)
            q_interp = self.slerp(eef_orient_quat, self.goal_quat, t=0.1)
            orient_error = q_interp - eef_orient_quat
            # self.error_orient = self.quaternion_to_euler(orient_error)
            self.error_orient = [0.0, 0.0, 0.0, 0.0]
            eef_posit = np.array([self.eef_pose.p[0], self.eef_pose.p[1], self.eef_pose.p[2]])
            self.error_posit = self.goal_posit - eef_posit

            # Print velocity for iai_naive_kinematics_sim
            self.pub_velocity.publish(velocity_msg)
            self.pub_clock.publish(clock)

            print 'joint_vel', Opt
            print '\n iter: ', i
            # print 'EEF: ', eef_orient_quat
            # print 'Goal: ', self.goal_quat
            # print 'Slerp:', q_interp
            # print 'error:', orient_error
            print 'error: ', self.error_posit
            # print 'Goal: ', self.goal_posit
            # print 'Now: ', eef_posit

        # rospy.spin()

    #def kinem_chain(self, name_frame_end, name_frame_base='base_link'):
    def kinem_chain(self, name_frame_end, name_frame_base='odom'):
        # Transform URDF to Chain() for the joints between 'name_frame_end' and 'name_frame_base'
        self.chain = kdl.Chain()

        try:
            self.joint_names = self.urdf_model.get_chain(name_frame_base, name_frame_end, links=False, fixed=False)
            self.name_frame_in = name_frame_base
            self.name_frame_out = name_frame_end

            # rospy.loginfo("Will control the following joints: %s" %(self.joint_names))

            self.kdl_tree = kdl_tree_from_urdf_model(self.urdf_model)
            self.chain = self.kdl_tree.getChain(name_frame_base, name_frame_end)
            self.kdl_fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
            self.kdl_ikv_solver = kdl.ChainIkSolverVel_wdls(self.chain)
            self.kdl_ikv_solver.setLambda(self.ik_lambda)
            self.nJoints = self.chain.getNrOfJoints()

            # Default Task and Joint weights
            self.tweights = np.identity(6)
            # weight matrix with 1 in diagonal to make use of all the joints.
            self.jweights = np.identity(self.nJoints)

            self.kdl_ikv_solver.setWeightTS(self.tweights.tolist())
            self.kdl_ikv_solver.setWeightJS(self.jweights.tolist())

            # Fill the list with the joint limits
            self.joint_limits_lower = np.empty(self.nJoints)
            self.joint_limits_upper = np.empty(self.nJoints)
            self.joint_vel_limits = np.empty(self.nJoints)

            for n, jnt_name in enumerate(self.joint_names):
                jnt = self.urdf_model.joint_map[jnt_name]
                if jnt.limit is not None:
                    if jnt.limit.lower is None:
                        self.joint_limits_lower[n] = -0.07
                    else:
                        self.joint_limits_lower[n] = jnt.limit.lower
                    if jnt.limit.upper is None:
                        self.joint_limits_upper[n] = -0.07
                    else:
                        self.joint_limits_upper[n] = jnt.limit.upper
                    self.joint_vel_limits[n] = jnt.limit.velocity

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
            #rospy.loginfo('Forward kinematics calculated')
            pass
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

    @staticmethod
    def rotation_to_quaternion(rot_matrix):
        w = math.sqrt(1+rot_matrix[0, 0] + rot_matrix[1, 1] + rot_matrix[2, 2])/2
        x = (rot_matrix[2, 1] - rot_matrix[1, 2])/(4*w)
        y = (rot_matrix[0, 2] - rot_matrix[2, 0])/(4*w)
        z = (rot_matrix[1, 0] - rot_matrix[0, 1])/(4*w)
        return np.array([x, y, z, w])

    @staticmethod
    def quaternion_to_euler(q):
        x, y, z, w = q
        roll = math.atan2(2 * (w * x + y * z), (1 - 2 * (x * x - y * y)))
        t2 = 2.0 * (w * y - z * x)
        t2 = 1 if t2 > 1 else t2
        t2 = -1 if t2 < -1 else t2
        pitch = math.asin(t2)
        t3 = 2 * (w * z + x * y)
        t4 = 1.0 - 2 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return np.array([roll, pitch, yaw])

    def slerp(self, q0, q1, t=1.0):
        # Interpolation between 2 quaternions, from 0 <= t <= 1
        dot_threshold = 0.9995
        q0_norm = self.normalize_quat(q0)
        q1_norm = self.normalize_quat(q1)
        dot = q0_norm[1]*q1_norm[1] + q0_norm[2]*q1_norm[2] + q0_norm[3]*q1_norm[3]

        # if quaternions are too close, do linear interpolation
        if abs(dot)>dot_threshold:
            q_slerp = q0_norm + t*(q1_norm-q0_norm)
            return q_slerp
        # if slerp wants to take the long path (>180 deg), change it
        if dot < 0.0:
            q1_norm = -q1_norm
            dot = -dot
        theta_0 = math.acos(dot)
        theta = theta_0*t
        q2 = q1_norm - q0_norm*dot
        q2_norm = self.normalize_quat(q2)

        return q0*math.cos(theta) + q2_norm*math.sin(theta)

    @staticmethod
    def normalize_quat(q):
        norm = math.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)
        q_norm = q / norm
        return q_norm

    def get_jacobian(self):
        # Obtain jacobian for the selected arm
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        jacobian = kdl.Jacobian(self.nJoints)
        self.jac_solver.JntToJac(self.joint_values_kdl, jacobian)

        jac_array = np.empty(0)
        for row in range(jacobian.rows()):
            for col in range(jacobian.columns()):
                jac_array = np.hstack((jac_array, jacobian[row,col]))
        jac_array = np.reshape(jac_array, (jacobian.rows(), jacobian.columns()))

        return jac_array

    def calculate_weigths(self):
        # Weigth of an active/inactive joint
        active_joint = 1e-4
        inact_joint = 10
        self.A[0, 0] = 1
        self.A[1, 1] = 1
        # Distange range of changing weight values
        a, b = 0.7, 1.1
        w_len = len(np.diag(self.jweights))
        jweights = np.ones(w_len)

        # Find bigger distance to goal (x or y)
        if abs(self.error_posit[0]) > abs(self.error_posit[1]):
            dist = abs(self.error_posit[0])
        else:
            dist = abs(self.error_posit[1])

        # If the robot is too far away, move only the base
        if dist >= b:
            jweights = np.ones(w_len)*inact_joint
            if abs(self.error_posit[0]) > b:
                jweights[0] = active_joint
            if abs(self.error_posit[1]) > b:
                jweights[1] = active_joint

        # Weights will go from 0.01 to 1, depending on the distance to the goal
        elif a < dist < b:
            # Base joints will have lower weights when far from the goal
            for n,w in enumerate(jweights):
                new_w = (inact_joint - active_joint)*(dist - a)/(b - a) + active_joint
                jweights[n] = new_w
            jweights[0] = (active_joint -inact_joint)*(dist - a)/(b - a) + inact_joint
            jweights[1] = (active_joint -inact_joint)*(dist - a)/(b - a) + inact_joint

        else:
            jweights = np.ones(w_len)*active_joint
            jweights[0] = inact_joint
            jweights[1] = inact_joint
            self.A[0,0] = 0
            self.A[1,1] = 0
            self.lbA[7], self.ubA[7] = 0, 0
            self.lbA[8], self.ubA[8] = 0, 0

        jweights[2] = 0
        self.jweights = np.diag(jweights)
        self.H = np.diag(np.hstack((np.diag(self.jweights), self.sweights)))
        print '--- Base weight: [%.4f, %.4f] \n--- Arm weight: %.4f'%(jweights[0],jweights[1],jweights[4])


if __name__ == '__main__':
    rospy.init_node('move_to_gp_server')
    rate = rospy.Rate(200)
    server = MoveToGPServer()
    rospy.spin()
