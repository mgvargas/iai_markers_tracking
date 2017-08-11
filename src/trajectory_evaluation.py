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
import math
import rospkg
import sys
import numpy as np
from iai_markers_tracking.srv import TrajectoryEvaluation, TrajectoryEvaluationResponse

def velocity_change(traj):
    trajectory = traj.trajectory
    joint_num = len(trajectory[0].velocity)
    old_vel =  np.zeros(joint_num)
    difference = []
    i = 0
    for x in trajectory:
        vel = np.array(x.velocity)
        difference.append(abs(old_vel - vel))
        old_vel = vel
        i += 1

    acc_sum =  np.zeros(joint_num)

    for step in difference:
        for x,joint in enumerate(step):
            acc_sum[x] += joint

    accel_average = [x/i for x in acc_sum]
    # Remove inactive joints
    acc_active = [x for x in accel_average if x]


    return np.mean(acc_active),difference, i


def acceleration_change(acceleration, iterat):
    joint_num = len(acceleration[0])
    jerk =  np.zeros(joint_num)
    old_acc =  np.zeros(joint_num)

    difference = []
    for acc in acceleration:
        difference.append(abs(old_acc - acc))
        old_acc = acc

    for step in difference:
        for x,joint in enumerate(step):
            jerk[x] += joint

    jerk_average = [x/iterat for x in jerk]
    jerk_active = [x for x in jerk_average if x]

    return np.mean(jerk_active)


def evaluation(traj):
    # Variable definition
    num = len(traj.trajectories)
    acc_change = []
    vel_change = []
    length = []
    grades = [0]*num

    # Calculate change in velocity, in acceleration and length of each trajectory
    for trajectory in traj.trajectories:
        vel, acc, l = velocity_change(trajectory)
        vel_change.append(vel)
        acc_change.append(acceleration_change(acc, l))
        length.append(l)

    print 'vel_change: ',vel_change
    print 'acc_change: ',acc_change
    print 'len: ',length

    min_vel = vel_change.index(min(vel_change))
    min_acc = acc_change.index(min(acc_change))
    min_length = length.index(min(length))
    grades[min_vel] += 1
    grades[min_acc] += 1
    grades[min_length] += 1
    selected_traj = grades.index(max(grades))

    print selected_traj

    return selected_traj
    #return TrajectoryEvaluationResponse(selected_traj)


def evaluate_traj():
    rospy.init_node('trajectory_evaluation_server')
    rospy.Service('trajectory_evaluation', TrajectoryEvaluation, evaluation)
    rospy.spin()


if __name__ == "__main__":
    evaluate_traj()