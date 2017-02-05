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
import math
import tf
import yaml
import rospkg

class ObjectGraspingMarker():
    grasp_poses = {}

    def __init__(self):
        self.s = rospy.Service('object_grasping_poses', ObjGrasping, self.list_grasping_poses)
        self.listener = tf.TransformListener(True, rospy.Duration(1))
        self.br = tf.TransformBroadcaster()
        self.markerArray = MarkerArray()
        self.yaml_file = {}
        self.trans_map = []
        self.rot_map = []
        self.grasp_poses = {}
        #pass

    # Read all frames published in /tf and finds markers
    def match_objects(self):
        frame_st = self.listener.getFrameStrings()
        print 'frame:',self.listener.getFrameStrings()
        matching = [s for s in frame_st if "marker" in s]
        return matching

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
        dir = rospack.get_path('rviz_markers') + '/config/database.yaml'
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

    # Get position of the camera
    def camera_pose(self):
        try:
            (self.trans_map, self.rot_map) = self.listener.lookupTransform('/map', '/camera_optical_frame', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        return self.trans_map, self.rot_map

    # Creates the markers for the objects (pose is wrt /map)
    def obj_pos_orient(self, obj):
        object = Marker()
        yaml_file = self.yaml_file

        for k in yaml_file.keys():
            if obj == k:  # For object in database
                a = yaml_file[k]['marker']
                for mar in a:
                    try:
                        # Pose of object wrt /map
                        (trans, rot_cam) = self.listener.lookupTransform('/camera_optical_frame', mar, rospy.Time(0))
                        object.pose.position.x = trans[0] - yaml_file[k][mar]['position'][0] + self.trans_map[0]
                        object.pose.position.y = trans[1] - yaml_file[k][mar]['position'][1] + self.trans_map[1]
                        object.pose.position.z = trans[2] - yaml_file[k][mar]['position'][2] + self.trans_map[2]
                        rot_marker = tuple(yaml_file[k][mar]['orientation'])
                        rot_ob_mark = ObjectGraspingMarker.q_inv(self, rot_marker)
                        rot1 = ObjectGraspingMarker.q_mult(self, rot_cam, rot_ob_mark)
                        rot2 = ObjectGraspingMarker.q_mult(self, rot1, self.rot_map)
                        object.pose.orientation.x = rot2[0]
                        object.pose.orientation.y = rot2[1]
                        object.pose.orientation.z = rot2[2]
                        object.pose.orientation.w = rot2[3]
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue
                    else: #If no exception
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
                        object.color.r = object.color.g = object.color.b = 0.6
                        object.color.a = 1.0
                        object.lifetime = rospy.Time(1)
                        poses = len(yaml_file[k]['grasping_poses'])

                        return object, poses

    # Create markers for each grasping pose of an object
    def find_poses(self, obj, n):
        mar = Marker()
        yaml_file = self.yaml_file
        for k in yaml_file.keys():
            if obj == k:
                # Marker properties
                mar.header.frame_id = obj
                mar.header.stamp = rospy.Time.now()
                mar.ns = obj + '_' + yaml_file[k]['grasping_poses'][n]['p_id']
                mar.id = yaml_file[k]['id'] * 100 + n
                mar.type = mar.ARROW
                mar.action = mar.ADD
                mar.mesh_use_embedded_materials = True
                mar.scale.x = 0.1
                mar.scale.y = mar.scale.z = 0.02
                mar.color.g = 0.3
                mar.color.r = mar.color.b = 0.9
                mar.color.a = 1.0
                mar.lifetime = rospy.Time(1)
                # Marker pose
                pos = yaml_file[k]['grasping_poses'][n]['position']
                orien = yaml_file[k]['grasping_poses'][n]['orientation']
                euler = tf.transformations.euler_from_quaternion(orien)
                z = math.copysign(math.cos(euler[1]) * mar.scale.x, pos[2])
                mar.pose.position.x = math.copysign(math.cos(euler[2]) * z, pos[0]) + pos[0]
                mar.pose.position.y = math.copysign(math.sin(euler[2]) * z, pos[1]) + pos[1]
                mar.pose.position.z = math.copysign(math.sin(euler[1]) * mar.scale.x, pos[2]) + pos[2]
                mar.pose.orientation.x = orien[0]
                mar.pose.orientation.y = orien[1]
                mar.pose.orientation.z = orien[2]
                mar.pose.orientation.w = orien[3]
                return mar, pos[0], pos[1], pos[2]

    # Published the /tf for all objects and the markers (of the grasping poses)
    def publish_obj(self, obj_list):
        found_obj = {}
        markers = {}
        for obj in obj_list:
            found_obj[obj] = Marker()
            self.grasp_poses[obj] = []
            (found_obj[obj], poses) = ObjectGraspingMarker.obj_pos_orient(self, obj)
            ob_pose = found_obj[obj].pose

            # Publish tf
            self.br.sendTransform((ob_pose.position.x, ob_pose.position.y, ob_pose.position.z),
                             (ob_pose.orientation.x, ob_pose.orientation.y, ob_pose.orientation.z,
                              ob_pose.orientation.w),
                             rospy.Time.now(), obj, "map")

            # Create markers for the grasping poses
            for n in range(poses):
                markers[n] = Marker()
                (markers[n], x, y, z) = ObjectGraspingMarker.find_poses(self, obj, n)
                self.grasp_poses[obj].append(markers[n].ns)

                self.br.sendTransform((x, y, z),
                                 (markers[n].pose.orientation.x, markers[n].pose.orientation.y,
                                  markers[n].pose.orientation.z, markers[n].pose.orientation.w),
                                 rospy.Time.now(),
                                 markers[n].ns, obj)

        return  markers, found_obj

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
    rospy.init_node('object_loader', anonymous=True)
    r = rospy.Rate(1)
    cl = ObjectGraspingMarker()

    marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=5)

    while not rospy.is_shutdown():
        markerArray = MarkerArray()
        matching = []

        # Find frames that are object markers
        matching = cl.match_objects()
        # Get position of the camera
        cl.camera_pose()
        # Open database YAML file
        cl.op_file()

        # Check if the objects are registered in the data base
        obj_list = cl.find_obj(matching)

        # Get the transforms from the objects to the map frame
        (markers, found_obj) = cl.publish_obj(obj_list)

        for n in markers:
            markerArray.markers.append(markers[n])

        for obj in found_obj:
            markerArray.markers.append(found_obj[obj])

        # Publish a table, just for visualization
        table = cl.create_table()
        markerArray.markers.append(table)

        print matching

        marker_pub.publish(markerArray)
        r.sleep()

    rospy.spin()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
