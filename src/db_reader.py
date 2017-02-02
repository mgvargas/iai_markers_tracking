#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from rviz_markers.msg import Object
import math
import tf
import roslib
import copy
import yaml
import rospkg
global listener

# Quaternion multiplication (input as tuples)
def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return x, y, z, w

# Quaternion inverse
def q_inv(q1):
    x1, y1, z1, w1 = q1
    denom = math.pow(w1,2) + math.pow(x1,2) + math.pow(y1,2) + math.pow(z1,2)
    w = w1 / denom
    x = -x1 / denom
    y = -y1 / denom
    z = -z1 / denom
    return x, y, z, w

# Find objects corresponding to perceived markers
def find_obj(matching, yaml_file):
    obj_list =[]
    for mark in matching:
        for y in yaml_file.keys():
            a = yaml_file[y]['marker']
            for n in a:
                if n == mark:
                    obj_list.append(y)
    return obj_list

# Publish position of the objects wrt map
def obj_pos_orient(obj, yaml_file, trans_map, rot_map):
    global listener
    object = Marker()

    for k in yaml_file.keys():
        if obj == k: # For object in database
            a = yaml_file[k]['marker']
            for mar in a:
                try:
                    # Pose of object wrt /map
                    (trans,rot_cam) = listener.lookupTransform('/camera_optical_frame', mar, rospy.Time(0))
                    object.pose.position.x = trans[0] - yaml_file[k][mar]['position'][0] + trans_map[0]
                    object.pose.position.y = trans[1] - yaml_file[k][mar]['position'][1] + trans_map[1]
                    object.pose.position.z = trans[2] - yaml_file[k][mar]['position'][2] + trans_map[2]
                    rot_marker = tuple (yaml_file[k][mar]['orientation'])
                    rot_ob_mark = q_inv(rot_marker)
                    rot1 = q_mult(rot_cam, rot_ob_mark)
                    rot2 = q_mult(rot1, rot_map)
                    object.pose.orientation.x = rot2[0]
                    object.pose.orientation.y = rot2[1]
                    object.pose.orientation.z = rot2[2]
                    object.pose.orientation.w = rot2[3]

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue
            # Object properties
            object.header.frame_id = 'map'
            object.header.stamp = rospy.Time.now()
            object.ns = obj
            object.id = yaml_file[k]['id']
            object.type = object.MESH_RESOURCE
            object.action = object.ADD
            object.mesh_resource = yaml_file[k]['mesh']
            object.mesh_use_embedded_materials = True
            object.scale.x = object.scale.y = object.scale.z = 1.0
            object.color.r = object.color.g = object.color.b = 0.6;
            object.color.a = 1.0
            object.lifetime = rospy.Time(0.2)
            poses = len(yaml_file[k]['grasping_poses'])

            return object, poses

# Create markers for each grasping pose of an object
def find_poses(obj_pose, obj, yaml_file, n):
    mar = Marker()
    for k in yaml_file.keys():
        if obj == k:
            # Object properties
            mar.header.frame_id = obj
            mar.header.stamp = rospy.Time.now()
            mar.ns = obj + '_' + yaml_file[k]['grasping_poses'][n]['p_id']
            mar.id = yaml_file[k]['id']*100 +n
            mar.type = mar.ARROW
            mar.action = mar.ADD
            mar.mesh_use_embedded_materials = True
            mar.scale.x = 0.1; mar.scale.y = mar.scale.z = 0.02
            mar.color.g = 0.3; mar.color.r = mar.color.b = 0.9;
            mar.color.a = 1.0
            mar.lifetime = rospy.Time(0.2)

            pos = yaml_file[k]['grasping_poses'][n]['position']
            orien = yaml_file[k]['grasping_poses'][n]['orientation']
            euler = tf.transformations.euler_from_quaternion(orien)
            z = math.copysign( math.cos(euler[1])*mar.scale.x, pos[2])
            mar.pose.position.x = math.copysign( math.cos(euler[2])*z, pos[0]) + pos[0]
            mar.pose.position.y = math.copysign( math.sin(euler[2])*z, pos[1]) + pos[1]
            mar.pose.position.z = math.copysign( math.sin(euler[1])*mar.scale.x, pos[2]) + pos[2]

            mar.pose.orientation.x = orien[0]
            mar.pose.orientation.y = orien[1]
            mar.pose.orientation.z = orien[2]
            mar.pose.orientation.w = orien[3]
            return mar, pos[0], pos[1], pos[2]

def main():
    global listener
    global br
    rospy.init_node('object_loader', anonymous=True)
    r = rospy.Rate(10)
    marker_pub = rospy.Publisher('visualization_marker_array',MarkerArray, queue_size=5)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        markerArray = MarkerArray()
        # Read all frames published in /tf
        frame_st = listener.getFrameStrings()

        # Find frames that are object markers
        # test_list = ['marker1', 'base_link', 'marker4', 'table', 'marker3']
        # matching = [s for s in test_list if "marker" in s]
        matching = [s for s in frame_st if "marker" in s]

        # Get position of the camera
        #trans_map= (1, 1, 2); rot_map.x = 0;rot_map.y = 0;rot_map.z = 0;rot_map.w = 1
        try:
            (trans_map,rot_map) = listener.lookupTransform('/map', '/camera_optical_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Open database YAML file
        rospack = rospkg.RosPack()
        dir = rospack.get_path('rviz_markers') + '/config/database.yaml'
        with open(dir, 'r') as f:
            try:
                db_file = f.read()
            except yaml.YAMLError as exc:
                print(exc)

        yaml_file = yaml.load(db_file) #Creates a dictionary

        # Check if the objects are registered in the data base
        obj_list = []
        obj_list = find_obj(matching, yaml_file)

        # Get the transforms from the objects to the map frame
        found_obj = {}
        markers = {}
        for obj in obj_list:
            found_obj[obj] = Marker()
            (found_obj[obj], poses) = obj_pos_orient(obj, yaml_file, trans_map, rot_map)
            markerArray.markers.append(found_obj[obj])
            ob_pose = found_obj[obj].pose

            # Publish tf
            br.sendTransform((ob_pose.position.x, ob_pose.position.y, ob_pose.position.z),
            (ob_pose.orientation.x, ob_pose.orientation.y, ob_pose.orientation.z, ob_pose.orientation.w),
            rospy.Time.now(),
            obj, "map")
            #print 'obj: ', obj
            #print found_obj[obj].pose.position

            # Create markers for the grasping poses
            for n in range(poses):
                markers[n] = Marker()
                (markers[n],x,y,z) = find_poses(found_obj[obj].pose, obj, yaml_file, n)
                markerArray.markers.append(markers[n])

                br.sendTransform(( x, y, z),
                (markers[n].pose.orientation.x, markers[n].pose.orientation.y, markers[n].pose.orientation.z, markers[n].pose.orientation.w),
                rospy.Time.now(),
                markers[n].ns, obj)


        # Publish a table, just for visualization
        table = Marker()
        table.header.frame_id = "/map";
        table.header.stamp = rospy.Time.now();
        table.ns = "kitchen_table";
        table.id = 0;
        table.type = table.MESH_RESOURCE;
        table.mesh_resource = "package://iai_kitchen/meshes/misc/big_table_1.dae"
        table.mesh_use_embedded_materials = True
        table.action = table.ADD;
        table.scale.x = table.scale.y = table.scale.z = 1.0;
        quaternion = tf.transformations.quaternion_from_euler(0, math.pi*0, 0)
        table.pose.position.x = 0;
        table.pose.position.y = 0
        table.pose.position.z = 0;
        table.pose.orientation.x = quaternion[0]
        table.pose.orientation.y = quaternion[1]
        table.pose.orientation.z = quaternion[2]
        table.pose.orientation.w = quaternion[3]
        table.scale.x = table.scale.y = table.scale.z = 1.0;
        table.color.r = table.color.g = 0.2; table.color.b = 0.6;
        table.color.a = 1.0;
        markerArray.markers.append(table)

        marker_pub.publish(markerArray)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
