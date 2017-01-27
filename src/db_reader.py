#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Pose
from rviz_markers.msg import object
import math
import tf
import roslib
import copy
import yaml
import rospkg

def find_markers(matching, yaml_file):
    ob_list =[]
    for obj in matching:
        for y in yaml_file.keys():
            a = yaml_file[y]['marker']
            for n in a:
                if n == obj:
                    ob_list.append(y)
    return ob_list


def main():

    rospy.init_node('object_loader', anonymous=True)
    r = rospy.Rate(100)
    marker_pub = rospy.Publisher('visualization_marker_array',MarkerArray, queue_size=5)
    object_pub = rospy.Publisher('visualization_object',object, queue_size=5)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        # Read all frames published in /tf
        frame_st = listener.getFrameStrings()

        # Find frames that are object markers
        test_list = ['marker1', 'base_link', 'marker4', 'table', 'marker3']
        matching = [s for s in test_list if "marker" in s]

        # Get the transforms from the objects to the camera frame
#        for x in matching:
#            try:
#                n = [int(s) for s in x if s.isdigit()] # Marker number
#                (trans[n],rot[n]) = listener.lookupTransform('/camera_optical_frame', x, rospy.Time(0))
#            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#                continue

        # tf test DELETE later
#        try:
#            (trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
#        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#            continue

        # Open database YAML file
        rospack = rospkg.RosPack()
        dir = rospack.get_path('rviz_markers') + '/config/database.yaml'
        with open(dir, 'r') as f:
            try:
                db_file = f.read()
            except yaml.YAMLError as exc:
                print(exc)

        yaml_file = yaml.load(db_file) #Creates a dictionary
        key = yaml_file.keys()

        # Check if the objects are registered in the data base, find pose markers
        obj_list = find_markers(matching, yaml_file)
        print obj_list



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
