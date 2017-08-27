// ============================================================================
// Name        : vectorfield_to_kinematic_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Receive the merged vectorfield and publish the steering vector
// ============================================================================

// ROS
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS - OpenCV_ Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/subscriber_filter.h>

#include "potential_field_utils.hpp"
#include <mutex>

using namespace std;

ros::Publisher pub;

static double meterPerPixel;
static int syncTopics;
static cv::Mat vectorfield;
static bool dataArrived = false;
static float velocityScale_meterPerSecond = 0.1;
static float angularScale_radPerSecond = 0.1;

void process(const cv::Mat &vectorfield, const nav_msgs::OdometryConstPtr odom) {

  cv::Point2i pose2d(pose2pixel(odom->pose.pose, vectorfield.cols, vectorfield.rows, meterPerPixel));
  if (pose2d.x < 0 || pose2d.x >= vectorfield.cols || pose2d.y < 0 || pose2d.y >= vectorfield.rows) {
    ROS_WARN("[%s] Current AMiRo pose2d %d %d is not in image.", ros::this_node::getName().c_str(), pose2d.x, pose2d.y);
    return;
  }

  // IMPORTANT: We assume the orientation of the robot resides in the world frame
  // Get the vector in the vectorfield at robot position
  cv::Point2f vector(vectorfield.at<cv::Vec2f>(pose2d.y, pose2d.x)[0], vectorfield.at<cv::Vec2f>(pose2d.y, pose2d.x)[1]);
  const float vectorAbs = sqrt(vector.dot(vector));
  cv::Point2f vectorUnit(vector.x / vectorAbs, vector.y / vectorAbs);
  const double vectorAngle = atan2(vector.x, vector.y);

  // Get the robot information
  const double robotAngle = tf::getYaw(odom->pose.pose.orientation);

  // Calculate the steering vector
  geometry_msgs::Twist twist;
  twist.linear.x = velocityScale_meterPerSecond * vectorAbs;
  float angleDiff = getAngleDiff(robotAngle, vectorAngle);
  angleDiff = angleDiff > M_PI ? 2 * M_PI - angleDiff : angleDiff;
  twist.angular.z = angularScale_radPerSecond * angleDiff;
  ROS_DEBUG_STREAM(ros::this_node::getName() << " VectorAbs: "<< vectorAbs << ", robotAngle: "<< robotAngle *180/M_PI << ", vectorAngle: " << vectorAngle * 180/M_PI << ", diff: " << angleDiff * 180/M_PI);
  pub.publish(twist);
}

void processSynced(const sensor_msgs::ImageConstPtr &image, const nav_msgs::OdometryConstPtr odom) {
  cv::Mat vectorfield = cv_bridge::toCvShare(image, image->encoding)->image;
  process(vectorfield, odom);
}

void processVectorfield(const sensor_msgs::ImageConstPtr& msg) {
  vectorfield = cv_bridge::toCvCopy(msg, msg->encoding)->image;
  dataArrived = true;
}

void processOdom(const nav_msgs::OdometryConstPtr &odom) {
  if (!dataArrived) {
    ROS_INFO_STREAM_ONCE(ros::this_node::getName() << " Waiting on vectorfield to arrive");
    return;
  } else {
    ROS_INFO_STREAM_ONCE(ros::this_node::getName() << " Vectorfield arrived");
  }
  process(vectorfield, odom);
}


int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", ros::this_node::getName().c_str());
  // Init ROS
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");

  string twistPublisherTopic;
  string vectorfield_listener_topic;
  string amiroOdomListenerTopic;
  node.param<string>("vectorfield_listener_topic", vectorfield_listener_topic, "/image/vectorfield");
  node.param<string>("amiro_odom_listener_topic", amiroOdomListenerTopic, "/amiro1/odom");
  node.param<string>("twist_publisher_topic", twistPublisherTopic, "/amiro1/cmd_vel");
  node.param<double>("meter_per_pixel", meterPerPixel, 0.003);
  node.param<float>("velocityScale_meterPerSecond", velocityScale_meterPerSecond, 0.1);
  node.param<float>("angularScale_radPerSecond", angularScale_radPerSecond, 0.1);
  node.param<int>("syncTopics", syncTopics, 0);
  ROS_INFO("[%s] vectorfield_listener_topic: %s", ros::this_node::getName().c_str(), vectorfield_listener_topic.c_str());
  ROS_INFO("[%s] amiro_odom_listener_topic: %s", ros::this_node::getName().c_str(), amiroOdomListenerTopic.c_str());
  ROS_INFO("[%s] twist_publisher_topic: %s", ros::this_node::getName().c_str(), twistPublisherTopic.c_str());
  ROS_INFO("[%s] meter_per_pixel: %f", ros::this_node::getName().c_str(), meterPerPixel);

  // The twist publisher
  pub = node.advertise<geometry_msgs::Twist>(twistPublisherTopic, 1);

  // The subscriber
  image_transport::ImageTransport imageTransport(node);
  image_transport::SubscriberFilter merged_potentialfield_sub;
  message_filters::Subscriber<nav_msgs::Odometry> amiro_odom_sub;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), merged_potentialfield_sub, amiro_odom_sub);
  image_transport::Subscriber vectorfield_sub;
  ros::Subscriber odom_sub;
  if (syncTopics) {
    merged_potentialfield_sub.subscribe(imageTransport, vectorfield_listener_topic, 1);
    amiro_odom_sub.subscribe(node, amiroOdomListenerTopic, 1);
    sync.registerCallback(boost::bind(&processSynced, _1, _2));
  } else {
    vectorfield_sub = imageTransport.subscribe(vectorfield_listener_topic, 1, &processVectorfield);
    odom_sub = node.subscribe(amiroOdomListenerTopic, 1, &processOdom);
  }

  ros::spin();
  return 0;
}
