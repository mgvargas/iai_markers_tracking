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
import tf
import tf2_ros
import yaml
import rospkg
import copy
from geometry_msgs.msg import TransformStamped
from tf.msg import tfMessage
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from iai_markers_tracking.msg import Object
from iai_markers_tracking.srv import GetObjectInfo
from apscheduler.schedulers.background import BackgroundScheduler


class ObjectGraspingMarker:
    grasp_poses = {}

    def __init__(self):
        rospy.init_node('object_db', anonymous=True)
        self.s = rospy.Service('get_object_info', GetObjectInfo, self.list_grasping_poses)
        self.tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfBuffer)
        self.tf_lis = rospy.Subscriber("tf", tfMessage, self.callback)
        self.camera_lis = rospy.Subscriber("/camera/camera_info", CameraInfo, self.callback_camera)
        self.object_pub = rospy.Publisher('detected_objects', Object, queue_size=20)
        self.br = tf2_ros.TransformBroadcaster()
        self.s_br = tf2_ros.StaticTransformBroadcaster()

        # Variable initialization
        self.markerArray = MarkerArray()
        self.frame_st = {'map'}
        self.yaml_file = {}
        self.grasp_poses = {}
        self.matching = []

    def callback(self, data):
        # Get the list of frames published by tf
        str_data = data.transforms[0].child_frame_id
        self.frame_st.update({str_data})

    def callback_camera(self, data):
        # Obtains the name of the camera frame
        self.camera_frame = data.header.frame_id

    def clear_v(self):
        self.frame_st = {}
        self.frame_st = {'map'}

    # Read all frames published in /tf and finds markers
    def match_objects(self):
        old = self.matching
        match = [s for s in self.frame_st.copy() if "tag_" in s]
        if len(match) > 0:
            self.matching = match
        else:
            self.matching = old
        return self.matching

    # Quaternion multiplication (input as tuples)
    @staticmethod
    def q_mult(q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return x, y, z, w

    # Quaternion inverse
    @staticmethod
    def q_inv(q1):
        x1, y1, z1, w1 = q1
        denom = math.pow(w1, 2) + math.pow(x1, 2) + math.pow(y1, 2) + math.pow(z1, 2)
        w = w1 / denom
        x = -x1 / denom
        y = -y1 / denom
        z = -z1 / denom
        return x, y, z, w

    # Open database YAML file
    def op_file(self):
        rospack = rospkg.RosPack()
        dir = rospack.get_path('iai_markers_tracking') + '/config/database.yaml'
        with open(dir, 'r') as f:
            try:
                db_file = f.read()
                self.yaml_file = yaml.load(db_file)  # Creates a dictionary
            except yaml.YAMLError as exc:
                print(exc)

    # Find objects corresponding to perceived markers (match database and markers)
    def find_obj(self, matching):
        obj_list = []
        for mark in matching:
            for y in self.yaml_file.keys():
                a = self.yaml_file[y]['marker']
                for n in a:
                    if n == mark:
                        obj_list.append(y)
        self.object_pub.publish(obj_list)
        return obj_list

    # Creates the object as a Marker() (pose is wrt /camera_optical_frame)
    def obj_pos_orient(self, obj):
        found_object = Marker()
        poses = 0

        for k in self.yaml_file.keys():
            if obj == k:  # For object in database
                a = self.yaml_file[k]['marker']
                for mar in a:
                    for tag in self.matching:
                        if tag == mar:  # if marker in object = marker found
                            rot_marker = self.yaml_file[k][mar]['orientation']
                            pos_marker = self.yaml_file[k][mar]['position']

                            # Publish object tf
                            t = TransformStamped()
                            t.header.stamp = rospy.Time.now()
                            t.header.frame_id = mar
                            t.child_frame_id = obj
                            t.transform.translation.x = pos_marker[0]
                            t.transform.translation.y = pos_marker[1]
                            t.transform.translation.z = pos_marker[2]
                            t.transform.rotation.x = rot_marker[0]
                            t.transform.rotation.y = rot_marker[1]
                            t.transform.rotation.z = rot_marker[2]
                            t.transform.rotation.w = rot_marker[3]
                            self.br.sendTransform(t)
                            try:
                                # Pose of object wrt /camera
                                t_cam = self.tfBuffer.lookup_transform(self.camera_frame, obj, rospy.Time())

                            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                                    tf2_ros.ExtrapolationException) as exc:
                                print '\nNo TF found, returning empty marker\n', exc
                                return found_object, poses
                            # If no exception
                            else:
                                # Object marker properties
                                found_object.pose.orientation = t_cam.transform.rotation
                                found_object.pose.position = t_cam.transform.translation
                                found_object.header.frame_id = self.camera_frame
                                found_object.header.stamp = rospy.Time.now()
                                found_object.ns = obj
                                found_object.id = self.yaml_file[k]['id']
                                found_object.type = found_object.MESH_RESOURCE
                                found_object.action = found_object.ADD
                                found_object.mesh_resource = self.yaml_file[k]['mesh']
                                found_object.mesh_use_embedded_materials = True
                                found_object.scale.x = found_object.scale.y = found_object.scale.z = self.yaml_file[k][
                                    'scale']
                                found_object.color.r = found_object.color.g = found_object.color.b = 0
                                found_object.color.a = 0
                                found_object.lifetime = rospy.Time(1)
                                poses = len(self.yaml_file[k]['grasping_poses'])

                            return found_object, poses

    # Create markers for each grasping pose of an object
    def poses_markers(self, obj, n):
        mar = Marker()

        for k in self.yaml_file.keys():
            if obj == k:
                # Marker properties
                mar.header.frame_id = obj
                mar.header.stamp = rospy.Time.now()
                mar.ns = obj + '_' + self.yaml_file[k]['grasping_poses'][n]['p_id']
                mar.id = self.yaml_file[k]['id'] * 100 + n
                mar.type = mar.MESH_RESOURCE
                mar.action = mar.ADD
                mar.color.a = 0.5
                mar.lifetime = rospy.Time(1)
                mar.color.g = 0.5
                mar.color.r = mar.color.b = 0.7
                mar.scale.x = mar.scale.y = mar.scale.z = 0.01
                # Marker pose
                pos = self.yaml_file[k]['grasping_poses'][n]['position']
                orien = self.yaml_file[k]['grasping_poses'][n]['orientation']
                mar.pose.position.x = pos[0]
                mar.pose.position.y = pos[1]
                mar.pose.position.z = pos[2]
                mar.mesh_resource = 'package://iai_markers_tracking/meshes/gripper_base.stl'
                mar.mesh_use_embedded_materials = True
                mar.pose.orientation.x = orien[0]
                mar.pose.orientation.y = orien[1]
                mar.pose.orientation.z = orien[2]
                mar.pose.orientation.w = orien[3]
                finger1 = copy.deepcopy(mar)
                finger1.mesh_resource = 'package://iai_markers_tracking/meshes/gripper_finger.stl'
                finger1.ns = mar.ns + '_f1'
                finger1.id = self.yaml_file[k]['id'] * 1000 + n
                finger1.header.frame_id = obj + '_' + self.yaml_file[k]['grasping_poses'][n]['p_id']
                finger1.pose.position.x = -(self.yaml_file[k]['gripper_opening']) / 2
                finger1.pose.position.y = 0
                finger1.pose.position.z = 0
                finger1.pose.orientation.x = 0
                finger1.pose.orientation.y = 0
                finger1.pose.orientation.z = 0
                finger1.pose.orientation.w = 1
                finger2 = copy.deepcopy(finger1)
                finger2.pose.position.x = (self.yaml_file[k]['gripper_opening']) / 2
                finger2.pose.position.y = 0.005
                finger2.pose.orientation.z = 1
                finger2.pose.orientation.w = 0
                finger2.ns = mar.ns + '_f2'
                finger1.id = self.yaml_file[k]['id'] * 2000 + n
                return mar, pos[0], pos[1], pos[2], orien, finger1, finger2

    # Published the /tf for all objects and the markers (of the grasping poses)
    def publish_obj(self, obj_list):
        found_obj = {}
        markers = {}
        finger1 = {}
        finger2 = {}
        for obj in obj_list:
            markers[obj] = {}
            finger1[obj] = {}
            finger2[obj] = {}
            pose_list = []
            (found_obj[obj], poses) = ObjectGraspingMarker.obj_pos_orient(self, obj)

            # Create markers for the grasping poses
            now = rospy.Time.now()
            for n in range(poses):
                # Markers for gripper base, left and right finger
                (markers[obj][n], x, y, z, orien, finger1[obj][n], finger2[obj][n]) = \
                    ObjectGraspingMarker.poses_markers(self, obj, n)
                pose_list.append(markers[obj][n].ns)

                # Marker TF (static transformation)
                static_tf = TransformStamped()
                static_tf.header.stamp = now
                static_tf.header.frame_id = obj
                static_tf.child_frame_id = markers[obj][n].ns
                static_tf.transform.translation.x = x
                static_tf.transform.translation.y = y
                static_tf.transform.translation.z = z
                static_tf.transform.rotation.w = orien[0]
                static_tf.transform.rotation.x = orien[1]
                static_tf.transform.rotation.y = orien[2]
                static_tf.transform.rotation.z = orien[3]
                
                self.s_br.sendTransform(static_tf)

            if len(pose_list) > 0:
                self.grasp_poses[obj] = pose_list

        return markers, found_obj, finger1, finger2

    #  ROS service for getting the name of the grasping poses of an object
    def list_grasping_poses(self, m):
        for n in self.grasp_poses:
            if n == m.object:
                g_p = [self.grasp_poses[m.object]]
                return g_p
            continue

    @staticmethod
    def create_table():
        table = Marker()
        table.header.frame_id = "/map"
        table.header.stamp = rospy.Time.now()
        table.ns = "kitchen_table"
        table.id = 0
        table.type = table.MESH_RESOURCE
        table.mesh_resource = "package://iai_kitchen/meshes/misc/big_table_1.dae"
        table.mesh_use_embedded_materials = True
        table.action = table.ADD
        table.lifetime = rospy.Time(2)
        table.scale.x = table.scale.y = table.scale.z = 1.0
        quaternion = tf.transformations.quaternion_from_euler(0, math.pi * 0, 0)
        table.pose.position.x = 0
        table.pose.position.y = 0
        table.pose.position.z = 0
        table.pose.orientation.x = quaternion[0]
        table.pose.orientation.y = quaternion[1]
        table.pose.orientation.z = quaternion[2]
        table.pose.orientation.w = quaternion[3]
        table.scale.x = table.scale.y = table.scale.z = 1.0
        table.color.r = table.color.g = 0.2
        table.color.b = 0.6
        table.color.a = 1.0
        return table


# Main function
def main():
    cl = ObjectGraspingMarker()
    r = rospy.Rate(5)
    rospy.sleep(0.5)
    scheduler = BackgroundScheduler()
    scheduler.add_job(cl.clear_v, 'interval', seconds=0.5)
    scheduler.start()

    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=5)

    # Open database YAML file
    cl.op_file()

    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        # Find frames that are object markers
        matching = cl.match_objects()

        # Check if the objects are registered in the data base
        obj_list = cl.find_obj(matching)

        # Get the transforms from the objects to the map frame
        (markers, found_obj, finger1, finger2) = cl.publish_obj(obj_list)

        for obj in found_obj:
            marker_array.markers.append(found_obj[obj])
            for n in markers[obj]:
                marker_array.markers.append(markers[obj][n])
                marker_array.markers.append(finger1[obj][n])
                marker_array.markers.append(finger2[obj][n])

        # Publish a table, just for visualization
        table = cl.create_table()
        marker_array.markers.append(table)

        marker_pub.publish(marker_array)
        r.sleep()

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass