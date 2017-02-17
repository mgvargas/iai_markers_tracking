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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from rviz_markers.srv import ObjGrasping
from tf.msg import tfMessage
from sensor_msgs.msg import CameraInfo
from apscheduler.schedulers.background import BackgroundScheduler
import math
import tf
import yaml
import rospkg
import copy

class ObjectGraspingMarker():
    grasp_poses = {}

    def __init__(self):
        rospy.init_node('object_loader', anonymous=True)
        self.s = rospy.Service('object_grasping_poses', ObjGrasping, self.list_grasping_poses)
        self.listener = tf.TransformListener(False, rospy.Duration(1))
        self.tf_lis = rospy.Subscriber("tf", tfMessage, self.callback)
        self.camera_lis = rospy.Subscriber("/camera/camera_info", CameraInfo, self.callback_camera)
        self.br = tf.TransformBroadcaster()
        self.markerArray = MarkerArray()
        self.frame_st = {'map'}
        self.yaml_file = {}
        self.grasp_poses = {}

    def callback(self, data):
        str_data = data.transforms[0].child_frame_id
        self.frame_st.update({str_data})

    def callback_camera(self, data):
        self.camera_frame = data.header.frame_id

    def clear_v(self):
        self.frame_st = {}
        self.frame_st = {'map'}

    # Read all frames published in /tf and finds markers
    def match_objects(self):
        self.matching = [s for s in self.frame_st if "tag_" in s]
        return self.matching

    # Quaternion multiplication (input as tuples)
    def q_mult(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return x, y, z, w

    # Quaternion inverse
    def q_inv(self, q1):
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
            except yaml.YAMLError as exc:
                print(exc)

        self.yaml_file = yaml.load(db_file)  # Creates a dictionary
        return self.yaml_file

    # Find objects corresponding to perceived markers (match database and markers)
    def find_obj(self, matching):
        obj_list = []
        for mark in matching:
            for y in self.yaml_file.keys():
                a = self.yaml_file[y]['marker']
                for n in a:
                    if n == mark:
                        obj_list.append(y)
        return obj_list

    # Creates the object as a Marker() (pose is wrt /camera_optical_frame)
    def obj_pos_orient(self, obj):
        found_object = Marker()
        poses = 0
        yaml_file = self.yaml_file

        for k in yaml_file.keys():
            if obj == k:  # For object in database
                a = yaml_file[k]['marker']
                for mar in a:
                    for tag in self.matching:
                        if tag == mar: # if marker in object = marker found
                            rot_marker = yaml_file[k][mar]['orientation']
                            pos_marker = yaml_file[k][mar]['position']
                            # Publish object tf
                            self.br.sendTransform((pos_marker[0], pos_marker[1], pos_marker[2]),
                                                  (rot_marker[0], rot_marker[1], rot_marker[2], rot_marker[3]),
                                                  rospy.Time.now(), obj, mar)
                            try:
                                # Pose of object wrt /camera
                                (trans, rot_cam) = self.listener.lookupTransform(self.camera_frame, obj, rospy.Time(0))
                            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                                return found_object, poses
                            # If no exception
                            else:
                                # Object marker properties
                                found_object.pose.position.x = trans[0]
                                found_object.pose.position.y = trans[1]
                                found_object.pose.position.z = trans[2]
                                found_object.pose.orientation.x = rot_cam[0]
                                found_object.pose.orientation.y = rot_cam[1]
                                found_object.pose.orientation.z = rot_cam[2]
                                found_object.pose.orientation.w = rot_cam[3]
                                found_object.header.frame_id = self.camera_frame
                                found_object.header.stamp = rospy.Time.now()
                                found_object.ns = obj
                                found_object.id = yaml_file[k]['id']
                                found_object.type = found_object.MESH_RESOURCE
                                found_object.action = found_object.ADD
                                found_object.mesh_resource = yaml_file[k]['mesh']
                                found_object.mesh_use_embedded_materials = True
                                found_object.scale.x = found_object.scale.y = found_object.scale.z = yaml_file[k]['scale']
                                found_object.color.r = found_object.color.g = found_object.color.b = 0
                                found_object.color.a = 0
                                found_object.lifetime = rospy.Time(1)
                                poses = len(yaml_file[k]['grasping_poses'])

                            return found_object, poses

    # Create markers for each grasping pose of an object
    def poses_markers(self, obj, n):
        mar = Marker()
        yaml_file = self.yaml_file
        for k in yaml_file.keys():
            if obj == k:
                # Marker properties
                mar.header.frame_id = obj
                mar.header.stamp = rospy.Time.now()
                mar.ns = obj + '_' + yaml_file[k]['grasping_poses'][n]['p_id']
                mar.id = yaml_file[k]['id'] * 100 + n
                #mar.type = mar.ARROW
                mar.type = mar.MESH_RESOURCE
                mar.action = mar.ADD
                mar.color.a = 0.2
                mar.lifetime = rospy.Time(1)
                # Marker pose
                pos = yaml_file[k]['grasping_poses'][n]['position']
                orien = yaml_file[k]['grasping_poses'][n]['orientation']
                if mar.type == mar.ARROW:
                    orien_a = yaml_file[k]['grasping_poses'][n]['orientation_arrow']
                    euler = tf.transformations.euler_from_quaternion(orien_a)
                    mar.color.g = 0.3
                    mar.color.r = mar.color.b = 0.9
                    mar.scale.x = 0.1
                    mar.scale.y = mar.scale.z = 0.02
                    z = math.copysign(math.cos(euler[1]) * (mar.scale.x+0.02), pos[2])
                    mar.pose.position.x = math.copysign(math.cos(euler[2]) * z, pos[0]) + pos[0]
                    mar.pose.position.y = math.copysign(math.sin(euler[2]) * z, pos[1]) + pos[1]
                    mar.pose.position.z = math.copysign(math.sin(euler[1]) * mar.scale.x, pos[2]) + pos[2]
                    mar.pose.orientation.x = orien_a[0]
                    mar.pose.orientation.y = orien_a[1]
                    mar.pose.orientation.z = orien_a[2]
                    mar.pose.orientation.w = orien_a[3]
                else:
                    mar.color.g = 0.5
                    mar.color.r = mar.color.b = 0.7
                    mar.scale.x = mar.scale.y = mar.scale.z = 0.01
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
                    finger1.id = yaml_file[k]['id'] * 1000 + n
                    finger1.header.frame_id = obj + '_' + yaml_file[k]['grasping_poses'][n]['p_id']
                    finger1.pose.position.x = -(yaml_file[k]['gripper_openning'])/2
                    finger1.pose.position.y = 0
                    finger1.pose.position.z = 0
                    finger1.pose.orientation.x = 0
                    finger1.pose.orientation.y = 0
                    finger1.pose.orientation.z = 0
                    finger1.pose.orientation.w = 1
                    finger2 = copy.deepcopy(finger1)
                    finger2.pose.position.x = (yaml_file[k]['gripper_openning']) / 2
                    finger2.pose.position.y = 0.005
                    finger2.pose.orientation.z = 1
                    finger2.pose.orientation.w = 0.00001
                    finger2.ns = mar.ns + '_f2'
                    finger1.id = yaml_file[k]['id'] * 2000 + n
                return mar, pos[0], pos[1], pos[2], orien, finger1, finger2

    # Published the /tf for all objects and the markers (of the grasping poses)
    def publish_obj(self, obj_list):
        found_obj = {}
        markers = {}
        finger1 ={}
        finger2 = {}
        for obj in obj_list:
            markers[obj] = {}
            finger1[obj] = {}
            finger2[obj] = {}
            found_obj[obj] = Marker()
            self.grasp_poses[obj] = []
            (found_obj[obj], poses) = ObjectGraspingMarker.obj_pos_orient(self, obj)

            # Create markers for the grasping poses
            for n in range(poses):
                markers[obj][n] = Marker()
                finger1[obj][n] = Marker()
                finger2[obj][n] = Marker()
                (markers[obj][n], x, y, z, orien, finger1[obj][n], finger2[obj][n]) = ObjectGraspingMarker.poses_markers(self, obj, n)
                self.grasp_poses[obj].append(markers[obj][n].ns)

                self.br.sendTransform((x, y, z),
                                 (orien[0], orien[1], orien[2], orien[3]),
                                 rospy.Time.now(),
                                 markers[obj][n].ns, obj)

        return markers, found_obj, finger1, finger2

    #  ROS service for getting the name of the grasping poses of an object
    def list_grasping_poses(self, m):
        for n in self.grasp_poses:
            if n == m.object:
                g_p = [self.grasp_poses[m.object]]
                return g_p

    def create_table(self):
        table = Marker()
        table.header.frame_id = "/map"
        table.header.stamp = rospy.Time.now()
        table.ns = "kitchen_table"
        table.id = 0
        table.type = table.MESH_RESOURCE
        table.mesh_resource = "package://iai_kitchen/meshes/misc/big_table_1.dae"
        table.mesh_use_embedded_materials = True
        table.action = table.ADD
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
    #rospy.init_node('object_loader', anonymous=True)
    cl = ObjectGraspingMarker()
    r = rospy.Rate(50)
    scheduler = BackgroundScheduler()
    scheduler.add_job(cl.clear_v, 'interval', seconds=0.5)
    scheduler.start()

    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=5)

    while not rospy.is_shutdown():
        markerArray = MarkerArray()

        # Find frames that are object markers
        matching = cl.match_objects()

        # Open database YAML file
        cl.op_file()

        # Check if the objects are registered in the data base
        obj_list = cl.find_obj(matching)

        # Get the transforms from the objects to the map frame
        (markers, found_obj, finger1, finger2) = cl.publish_obj(obj_list)

        for obj in found_obj:
            markerArray.markers.append(found_obj[obj])
            for n in markers[obj]:
                markerArray.markers.append(markers[obj][n])
                markerArray.markers.append(finger1[obj][n])
                markerArray.markers.append(finger2[obj][n])

        # Publish a table, just for visualization
        #table = cl.create_table()
        #markerArray.markers.append(table)

        marker_pub.publish(markerArray)
        r.sleep()

    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
