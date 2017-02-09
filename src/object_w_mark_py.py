#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import tf
import copy

def main():

    rospy.init_node('basic_shapes')
    r = rospy.Rate(10)
    marker_pub = rospy.Publisher('visualization_marker_array',MarkerArray, queue_size=1)
    # mesh_pub = rospy.Publisher('visualization_marker',Marker, queue_size=1)
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        marker = Marker()
        marker2 = Marker()
        mesh = Marker()
        markerArray = MarkerArray()

        #----- Object properties
        # Set the frame ID and timestamp.  See the TF tutorials for information on these.
        mesh.header.frame_id = "/map"
        mesh.header.stamp = rospy.Time.now()

        # Set the namespace and id for this marker.  This serves to create a unique ID
        # Any marker sent with the same namespace and id will overwrite the old one
        mesh.ns = "mesh_test"
        mesh.id = 0

        # Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        mesh.type = marker.MESH_RESOURCE
        mesh.mesh_resource = "package://iai_kitchen/meshes/misc/bowl.stl"
        mesh.mesh_use_embedded_materials = True

        # Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        mesh.action = marker.ADD
        mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0

        # Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        quaternion = tf.transformations.quaternion_from_euler(0, math.pi*0, math.pi/2)
        x = 0.39
        y = 0.31
        mesh.pose.position.x = 1.3
        mesh.pose.position.y = 1.85
        mesh.pose.position.z = -1.13
        mesh.pose.orientation.x = quaternion[0]
        mesh.pose.orientation.y = quaternion[1]
        mesh.pose.orientation.z = quaternion[2]
        mesh.pose.orientation.w = quaternion[3]

        # Set the scale of the marker -- 1x1x1 here means 1m on a side
        mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0
        mesh.lifetime = rospy.Time(1)

        # Set the color -- be sure to set alpha to something non-zero!
        mesh.color.r = mesh.color.g = mesh.color.b = 0.8
        mesh.color.a = 1.0

        #marker.lifetime = rospy.Duration();

        # General Markers properties
        marker.header.frame_id = "map"
        marker.lifetime = rospy.Time(1)
        marker.header.stamp = rospy.Time.now()
        marker.ns = "marker1"
        marker.id = 1
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.15
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker2 = copy.deepcopy(marker)
        marker2.ns = "marker2"
        marker2.id = 2
        marker3 = copy.deepcopy(marker)
        marker3.ns = "marker3"
        marker3.id = 3

        # Set the pose of the markers.
        offset = 0.05
        quaternion1 = tf.transformations.quaternion_from_euler(0, 0, 0)
        quaternion2 = tf.transformations.quaternion_from_euler(0, math.pi/6, math.pi/2)
        quaternion3 = tf.transformations.quaternion_from_euler(0, 0, 0*math.pi/2)
        marker.pose.position.x = - (x/2 + offset + marker.scale.x)*0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = quaternion1[0]
        marker.pose.orientation.y = quaternion1[1]
        marker.pose.orientation.z = quaternion1[2]
        marker.pose.orientation.w = quaternion1[3]
        marker2.pose.position.x = 1.0
        marker2.pose.position.y = 2.0
        marker2.pose.position.z = -1.12
        marker2.pose.orientation.x = quaternion2[0]
        marker2.pose.orientation.y = quaternion2[1]
        marker2.pose.orientation.z = quaternion2[2]
        marker2.pose.orientation.w = quaternion2[3]
        marker3.pose.position.x = + (x/2 + offset + marker.scale.x)
        marker3.pose.position.y = 0.0
        marker3.pose.position.z = 0.0
        marker3.pose.orientation.x = quaternion3[0]*0
        marker3.pose.orientation.y = quaternion3[1]*0
        marker3.pose.orientation.z = quaternion3[2]*0
        marker3.pose.orientation.w = quaternion3[3]*0 + 1

        br.sendTransform((marker2.pose.position.x, marker2.pose.position.y, marker2.pose.position.z),
                        (quaternion1[0], quaternion1[1], quaternion1[2], quaternion1[3]),
                        rospy.Time.now(),
                        "marker2",
                        "camera_optical_frame")

        # Create a frame for the object
        br.sendTransform((mesh.pose.position.x, mesh.pose.position.y, mesh.pose.position.z),
                         (quaternion2[0], quaternion2[1], quaternion2[2], quaternion2[3]),
                         rospy.Time.now(),
                         "marker4",
                         "camera_optical_frame")

        br.sendTransform((-1, -2, 2),
                         (quaternion3[0], quaternion3[1], quaternion3[2], quaternion3[3]),
                        rospy.Time.now(),
                        "camera_optical_frame","map" )


        # Publish the markers
        markerArray.markers.append(marker)
        markerArray.markers.append(marker2)
        markerArray.markers.append(marker3)
        markerArray.markers.append(mesh)
        #marker_pub.publish(markerArray)

        r.sleep()





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
