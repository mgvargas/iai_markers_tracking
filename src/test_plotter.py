#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
import rospkg
import sys
import yaml
import rosbag
# TODO: save bag file with date/time as name
from datetime import datetime, date
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class PlotTest:

    def __init__(self, r=0.0, g=0.4, b=1.0):
        self.r, self.g, self.b = r, g, b
        rospy.Subscriber('/data_to_plot', PoseArray, self.plot_callback)
        self.marker_pub = rospy.Publisher('eef_trajectory_marker_array', MarkerArray, queue_size=1)
        #self.marker_pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size=1)
        self.flag = False

    def plot_callback(self, pose_array):
        #self.pose_array = pose_array
        if len(pose_array.poses) > 0:
            if self.flag == False:
                self.write_bag(pose_array)
                self.create_markers(pose_array, self.r, self.g, self.b)
                self.flag = True

    def create_markers(self, pose_array, r, g, b):
        self.pose_array = pose_array
        markerArray = MarkerArray()

        for n,pose in enumerate(self.pose_array.poses):
            marker = Marker()
            marker.pose = pose
            marker.header.frame_id = "odom"
            marker.header.stamp = rospy.Time.now()
            marker.id = n*b*10
            marker.ns = "marker_" + str(n)
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.03
            marker.scale.y = 0.03
            marker.scale.z = 0.03
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 1.0

            markerArray.markers.append(marker)

        #self.yaml_writer(markerArray)
        self.marker_pub.publish(markerArray)
        # print 'r, g, b', r, g, b

    @staticmethod
    def yaml_writer(markerArray):
        # Write a YAML file with the parameters for the simulated controller
        try:
            # Open YAML configuration file
            pack = rospkg.RosPack()
            dir = pack.get_path('iai_markers_tracking') + '/test_plot_data/controller_param.yaml'

            data = markerArray

            # Write file
            with open(dir, 'w') as outfile:
                yaml.dump(data, outfile, default_flow_style=False)
        except yaml.YAMLError:
            rospy.logerr("Unexpected error while writing controller configuration YAML file:"), sys.exc_info()[0]
            return -1

    @staticmethod
    def write_bag(pose_array):
        pack = rospkg.RosPack()
        hoy = datetime.now()
        day = '-' + str(hoy.month) + '-' + str(hoy.day) + '_' + str(hoy.hour) + '-' + str(hoy.minute)
        dir = pack.get_path('iai_markers_tracking') + '/test_plot_data/test_2017' + day + '.bag'
        bag = rosbag.Bag(dir, 'w')
        try:
            bag.write('data_to_plot',pose_array)
        finally:
            bag.close()
        return 0

def main(r=0.0, g=0.4, b=1.0):
    print 'plotting trajectory'
    PlotTest(r, g, b)
    #play_bag()

    return 0


def play_bag():
    pt = PlotTest()
    pack = rospkg.RosPack()
    dir = pack.get_path('iai_markers_tracking') + '/test_plot_data/test.bag'
    bag = rosbag.Bag(dir)
    for topic, msg, t in bag.read_messages(topics=['data_to_plot']):
        print 'plot'
        pt.create_markers(msg)
    bag.close()


if __name__ == '__main__':
    try:
        rospy.init_node('plot_eef_trajectory')
        rate = rospy.Rate(200)
        main()
    except rospy.ROSInterruptException:
        pass
