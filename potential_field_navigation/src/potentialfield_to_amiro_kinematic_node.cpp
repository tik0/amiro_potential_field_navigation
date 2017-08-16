// ============================================================================
// Name        : potentialfield_to_amiro_kinematic_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve the merged potentialfield and merger them.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS - OpenCV_ Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.h>

using namespace std;

ros::Publisher pub;

// program name
const string programName = "potentialfield_to_amiro_kinematic_node";

double meterPerPixel;

void process(const sensor_msgs::ImageConstPtr &image, const nav_msgs::OdometryConstPtr odom) {
  cv::Mat potentialfield = cv_bridge::toCvShare(image, image->encoding)->image;

  cv::Point2i pose2d((int) (odom->pose.pose.position.x / meterPerPixel + potentialfield.rows / 2), (int) (odom->pose.pose.position.y / meterPerPixel + potentialfield.cols / 2));

  if (pose2d.x < 0 || pose2d.x >= potentialfield.rows || pose2d.y < 0 || pose2d.y > potentialfield.rows) {
    ROS_WARN("[%s] Current amiro pose2d %d %d is not in image.", programName.c_str(), pose2d.x, pose2d.y);
    return;
  }

  cv::Point2f potential(potentialfield.at<cv::Vec2f>(pose2d.y, pose2d.x)[0], potentialfield.at<cv::Vec2f>(pose2d.y, pose2d.x)[1]);
  double length = sqrt(potential.x * potential.x + potential.y * potential.y);
  double angle = atan2(pose2d.y, pose2d.x);

  geometry_msgs::Twist twist;
  twist.linear.x = length;
  twist.angular.z = angle;
  pub.publish(twist);
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());
  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  string twistPublisherTopic;
  string mergedPotentialFieldListenerTopic;
  string amiroOdomListenerTopic;
  node.param<string>("merged_potentialfield_listener_topic", mergedPotentialFieldListenerTopic, "/image/potentialfield");
  node.param<string>("amiro_odom_listener_topic", amiroOdomListenerTopic, "/amiro1/odom");
  node.param<string>("twist_publisher_topic", twistPublisherTopic, "/amiro1/cmd_vel");
  node.param<double>("meter_per_pixel", meterPerPixel, 0.003);
  ROS_INFO("[%s] merged_potentialfield_listener_topic: %s", programName.c_str(), mergedPotentialFieldListenerTopic.c_str());
  ROS_INFO("[%s] amiro_odom_listener_topic: %s", programName.c_str(), amiroOdomListenerTopic.c_str());
  ROS_INFO("[%s] twist_publisher_topic: %s", programName.c_str(), twistPublisherTopic.c_str());
  ROS_INFO("[%s] meter_per_pixel: %f", programName.c_str(), meterPerPixel);

  image_transport::ImageTransport imageTransport(node);

  image_transport::SubscriberFilter merged_potentialfield_sub(imageTransport, mergedPotentialFieldListenerTopic, 1);
  message_filters::Subscriber<nav_msgs::Odometry> amiro_odom_sub(node, amiroOdomListenerTopic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), merged_potentialfield_sub, amiro_odom_sub);
  sync.registerCallback(boost::bind(&process, _1, _2));


  pub = node.advertise<geometry_msgs::Twist>(twistPublisherTopic, 1);

  ros::spin();
  return 0;
}
