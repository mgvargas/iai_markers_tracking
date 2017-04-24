#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
namespace vm = visualization_msgs;

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
  ros::Publisher mesh_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    vm::Marker mesh;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    mesh.header.frame_id = "/map";
    mesh.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
    mesh.ns = "mesh_test";
    mesh.id = 1;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;
    mesh.type = vm::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://iai_boxy_torso/meshes/torso_base_link.dae";
    mesh.mesh_resource = "package://iai_boxy_torso/meshes/triangle_base_link.dae";

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
    mesh.action = vm::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    mesh.pose.position.x = 1;
    mesh.pose.position.y = 0;
    mesh.pose.position.z = 1;
    mesh.pose.orientation.x = 0.0;
    mesh.pose.orientation.y = 0.0;
    mesh.pose.orientation.z = 0.0;
    mesh.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.5;
    mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.7f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    mesh.color.r = mesh.color.g = mesh.color.b = 0.8;
    mesh.color.a = 1.0;

    marker.lifetime = ros::Duration();
    mesh.lifetime = ros::Duration();


    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    marker_pub.publish(mesh);

    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::MESH_RESOURCE;
      break;
    case visualization_msgs::Marker::MESH_RESOURCE:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

    r.sleep();
  }
}
