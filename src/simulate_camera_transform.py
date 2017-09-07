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
import math
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo


def main():

    rospy.init_node('test_publish_tags')
    r = rospy.Rate(15)
    camera_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=30)
    br = tf2_ros.TransformBroadcaster()

    while not rospy.is_shutdown():     
        camera = CameraInfo()
        camera.header.frame_id = 'camera_optical_frame'
        camera_pub.publish(camera)

        quaternion = tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0)

        now = rospy.Time.now()

        camera = TransformStamped()
        camera.header.stamp = now
        camera.header.frame_id = 'odom'
        camera.child_frame_id = 'camera_optical_frame'
        camera.transform.translation.x = -0.5
        camera.transform.translation.y = 0
        camera.transform.translation.z = 2
        camera.transform.rotation.x = quaternion[0]
        camera.transform.rotation.y = quaternion[1]
        camera.transform.rotation.z = quaternion[2]
        camera.transform.rotation.w = quaternion[3]
        
        br.sendTransform(camera)

        r.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
