#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import CameraInfo
import math
import tf
import copy

def main():

    rospy.init_node('test_publish_tags')
    r = rospy.Rate(50)
    camera_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=30)
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        quaternion1 = tf.transformations.quaternion_from_euler(math.radians(0), 0, 0)
        quaternion2 = tf.transformations.quaternion_from_euler(math.radians(5), math.radians(-5), 0)
        quaternion3 = tf.transformations.quaternion_from_euler(math.radians(-12), math.radians(5), 0)
        quaternion4 = tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0)

        br.sendTransform((0.6, 1.24, 0.2),
                         (quaternion1[0], quaternion1[1], quaternion1[2], quaternion1[3]),
                         rospy.Time.now(),
                         "tag_0", "camera_optical_frame")

        br.sendTransform((1.0, 1.21, -0.2),
                         (quaternion2[0], quaternion2[1], quaternion2[2], quaternion2[3]),
                         rospy.Time.now(),
                         "tag_1",
                         "camera_optical_frame")

        br.sendTransform((0.1, 1.18, 0.1),
                         (quaternion3[0], quaternion3[1], quaternion3[2], quaternion3[3]),
                         rospy.Time.now(),
                         "tag_3",
                         "camera_optical_frame")

        br.sendTransform((0.6, 1.22, -0.13),
                         (quaternion1[0], quaternion1[1], quaternion1[2], quaternion1[3]),
                         rospy.Time.now(),
                         "tag_5",
                         "camera_optical_frame")

        br.sendTransform((-0.5, 0, 2),
                         (quaternion4[0], quaternion4[1], quaternion4[2], quaternion4[3]),
                         rospy.Time.now(),
                         "camera_optical_frame", "map")

        camera = CameraInfo()
        camera.header.frame_id = 'camera_optical_frame'

        camera_pub.publish(camera)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass