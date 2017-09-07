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

        quaternion0 = tf.transformations.quaternion_from_euler(math.radians(0), math.radians(85), math.radians(0))
        quaternion1 = tf.transformations.quaternion_from_euler(math.radians(0), math.radians(90), math.radians(0))
        quaternion2 = tf.transformations.quaternion_from_euler(math.radians(5), math.radians(85), 0)
        quaternion3 = tf.transformations.quaternion_from_euler(math.radians(-10), math.radians(-90-15), 0)
        quaternion4 = tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0)

        now = rospy.Time.now()
        
        knorr = TransformStamped()
        knorr.header.stamp = now
        knorr.header.frame_id = camera.header.frame_id
        knorr.child_frame_id = 'tag_0'
        knorr.transform.translation.x = 0.25
        knorr.transform.translation.y = 1.24
        knorr.transform.translation.z = 0.2
        knorr.transform.rotation.x = quaternion0[0]
        knorr.transform.rotation.y = quaternion0[1]
        knorr.transform.rotation.z = quaternion0[2]
        knorr.transform.rotation.w = quaternion0[3]

        cup = TransformStamped()
        cup.header.stamp = now
        cup.header.frame_id = camera.header.frame_id
        cup.child_frame_id = 'tag_1'
        cup.transform.translation.x = 0.35
        cup.transform.translation.y = 1.21
        cup.transform.translation.z = -0.55
        cup.transform.rotation.x = quaternion2[0]
        cup.transform.rotation.y = quaternion2[1]
        cup.transform.rotation.z = quaternion2[2]
        cup.transform.rotation.w = quaternion2[3]

        bowl = TransformStamped()
        bowl.header.stamp = now
        bowl.header.frame_id = camera.header.frame_id
        bowl.child_frame_id = 'tag_3'
        bowl.transform.translation.x = 0.65
        bowl.transform.translation.y = 1.18
        bowl.transform.translation.z = 0.3
        bowl.transform.rotation.x = quaternion3[0]
        bowl.transform.rotation.y = quaternion3[1]
        bowl.transform.rotation.z = quaternion3[2]
        bowl.transform.rotation.w = quaternion3[3]

        mondamin = TransformStamped()
        mondamin.header.stamp = now
        mondamin.header.frame_id = camera.header.frame_id
        mondamin.child_frame_id = 'tag_5'
        mondamin.transform.translation.x = 0.5
        mondamin.transform.translation.y = 1.22
        mondamin.transform.translation.z = -0.13
        mondamin.transform.rotation.x = quaternion1[0]
        mondamin.transform.rotation.y = quaternion1[1]
        mondamin.transform.rotation.z = quaternion1[2]
        mondamin.transform.rotation.w = quaternion1[3]

        table = TransformStamped()
        table.header.stamp = now
        table.header.frame_id = camera.header.frame_id
        table.child_frame_id = 'tag_9'
        table.transform.translation.x = 0.0
        table.transform.translation.y = 1.22+0.428
        table.transform.translation.z = 0.0
        table.transform.rotation.x = quaternion1[0]
        table.transform.rotation.y = quaternion1[1]
        table.transform.rotation.z = quaternion1[2]
        table.transform.rotation.w = quaternion1[3]

        camera = TransformStamped()
        camera.header.stamp = now
        camera.header.frame_id = 'map'
        camera.child_frame_id = 'camera_optical_frame'
        camera.transform.translation.x = -0.5
        camera.transform.translation.y = 0
        camera.transform.translation.z = 2
        camera.transform.rotation.x = quaternion4[0]
        camera.transform.rotation.y = quaternion4[1]
        camera.transform.rotation.z = quaternion4[2]
        camera.transform.rotation.w = quaternion4[3]
        
        br.sendTransform(knorr)
        br.sendTransform(cup)
        br.sendTransform(bowl)
        br.sendTransform(mondamin)
        br.sendTransform(camera)


        r.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
