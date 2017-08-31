// ============================================================================
// Name        : pixel_odom_to_pseudo_metric_odom_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Recieve pixel odometry and publish it to pseudo metric odometry.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#include "potential_field_utils.hpp"

using namespace std;

//params
static double meterPerPixel;
static string frameId, targetFrameId;
tf::TransformListener* tfListener;
ros::Publisher pub;

void process(const nav_msgs::OdometryConstPtr &odom_pixel) {
  tf::StampedTransform transform;
  tf::StampedTransform transformBack;
  try {
    tfListener->lookupTransform(targetFrameId, frameId, ros::Time(0), transform);
    tfListener->lookupTransform(frameId, targetFrameId, ros::Time(0), transformBack);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("[%s] %s",ros::this_node::getName().c_str(), ex.what());
    return;
  }

  nav_msgs::Odometry odom_pseudo_metric;
  odom_pseudo_metric.header = odom_pixel->header;
  odom_pseudo_metric.header.frame_id = frameId;

  // Transform origin position into frameId transform.rotation
  tf::Vector3 vec3(odom_pixel->pose.pose.position.x * meterPerPixel, odom_pixel->pose.pose.position.y * meterPerPixel, 0);
  tf::Vector3 vec3_out = tf::quatRotate(transform.getRotation(), vec3);

  // Transform translation between frameId and targetFrameId with transform.rotation
  tf::Vector3 trans_vec3(transform.getOrigin().x(), transform.getOrigin().y(), 0);
  tf::Vector3 trans_vec3_out = tf::quatRotate(transform.getRotation(), trans_vec3);

  // Calc the resulting pose
  odom_pseudo_metric.pose.pose.position.x = vec3_out.x() - trans_vec3_out.x();
  odom_pseudo_metric.pose.pose.position.y = vec3_out.y() - trans_vec3_out.y();

  // Transform the origin rotation with the frameId transform.rotation
  tf::Quaternion q_odom;
  tf::quaternionMsgToTF(odom_pixel->pose.pose.orientation, q_odom);
  tf::quaternionTFToMsg(q_odom * transform.getRotation(), odom_pseudo_metric.pose.pose.orientation);

  odom_pseudo_metric.pose.covariance = odom_pixel->pose.covariance;

  pub.publish(odom_pseudo_metric);
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", ros::this_node::getName().c_str());
  // Init ROS
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");

  string amiroPixelOdomListenerTopic, amiroPseudoMetricOdomPublisherTopic;
  node.param<string>("amiro_pixel_odom_listener_topic", amiroPixelOdomListenerTopic, "/amiro1/odom/pixel");
  node.param<string>("amiro_pseudo_metric_odom_publisher_topic", amiroPseudoMetricOdomPublisherTopic, "/amiro1/odom/pseudo_metric");
  node.param<string>("frame_id", frameId, "world");
  node.param<string>("target_frame_id", targetFrameId, "cam");
  node.param<double>("meter_per_pixel", meterPerPixel, 0.003);
  ROS_INFO("[%s] amiro_pixel_odom_listener_topic: %s", ros::this_node::getName().c_str(), amiroPixelOdomListenerTopic.c_str());
  ROS_INFO("[%s] amiro_pseudo_metric_odom_publisher_topic: %s", ros::this_node::getName().c_str(), amiroPseudoMetricOdomPublisherTopic.c_str());
  ROS_INFO("[%s] frame_id: %s", ros::this_node::getName().c_str(), frameId.c_str());
  ROS_INFO("[%s] target_frame_id: %s", ros::this_node::getName().c_str(), targetFrameId.c_str());
  ROS_INFO("[%s] meter_per_pixel: %f", ros::this_node::getName().c_str(), meterPerPixel);

  tf::TransformListener lr(ros::Duration(10));
  tfListener = &lr;
  pub = node.advertise<nav_msgs::Odometry>(amiroPseudoMetricOdomPublisherTopic, 1);
  ros::Subscriber odom_sub = node.subscribe(amiroPixelOdomListenerTopic, 1, &process);

  ros::spin();
  return 0;

}
