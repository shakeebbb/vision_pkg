#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"

using namespace std;

ros::Publisher pose_pub;
ros::Subscriber pose_sub;

geometry_msgs::PoseStamped poseStamped;

void pose_cb(const geometry_msgs::TransformStamped&);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vicon_remap");

  ros::NodeHandle n;

  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 1000);
  pose_sub = n.subscribe("/vicon/quad3/quad3", 1000, pose_cb);

  ros::spin();
  return  0;
}

void pose_cb(const geometry_msgs::TransformStamped& transformStamped)
{
poseStamped.header = transformStamped.header;
poseStamped.pose.position.x = transformStamped.transform.translation.x;
poseStamped.pose.position.y = transformStamped.transform.translation.y;
poseStamped.pose.position.z = transformStamped.transform.translation.z;

poseStamped.pose.orientation = transformStamped.transform.rotation;

pose_pub.publish(poseStamped);
}
