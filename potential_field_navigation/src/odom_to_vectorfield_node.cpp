// ============================================================================
// Name        : odom_to_vectorfield_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Recieve odometry and creates a vectorfield.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
// ROS - OpenCV_ Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

#include "potential_field_utils.hpp"

using namespace std;

#define C 128
//params
float heuristic_abs_min, heuristic_factor;
double meterPerPixel;
int imageWidth;
int imageHeight;

// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisherPot, imagePublisherVec;

static cv::Point2i pose2dLast;
static bool firstRun = true;
static int minPoseDiff_pixel = 10;

void process(const nav_msgs::OdometryConstPtr &odom) {
  cv::Mat potentialField(imageHeight, imageWidth, CV_32FC1);
  // TODO Calculate with respect to correct corrdinate frame
  // cv::Point2i pose2d((int) (odom->pose.pose.position.x / meterPerPixel + imageWidth/2), (int) (odom->pose.pose.position.y / meterPerPixel + imageHeight/2));
  cv::Point2i pose2d((int) (imageWidth/2 - odom->pose.pose.position.y / meterPerPixel), (int) (imageHeight/2 - odom->pose.pose.position.x / meterPerPixel));


  if (firstRun) {
    firstRun = false;
    pose2dLast = pose2d;
  } else {
    // Check for movement, otherwise don't process
    if (sqrt(pow(pose2d.x - pose2dLast.x, 2) + pow(pose2d.y - pose2dLast.y, 2)) >= minPoseDiff_pixel) {
      pose2dLast = pose2d;
    } else {
      return;
    }
  }


  // Calculate the potential field
#pragma omp parallel for
  for (int y = 0; y < potentialField.rows; y++) {
    for (int x = 0; x < potentialField.cols; x++) {
      if (x != pose2d.x || y != pose2d.y) {
        potentialField.at<float>(y, x) = 1.0f / sqrt(pow(x-pose2d.x,2) + pow(y-pose2d.y,2));
      }
    }
  }

  // Get the vector field
  cv::Mat vectorField = potentialfield_to_vectorfield(potentialField);

  // Apply heuristics
  for (int idx = 0; idx < potentialField.rows * potentialField.cols; ++idx) {
    float &x = vectorField.at<cv::Vec2f>(idx)[0];
    float &y = vectorField.at<cv::Vec2f>(idx)[1];
    const float abs = sqrt(x*x + y*y);
    if (abs > heuristic_abs_min) {
      x = 0.0;
      y = 0.0;
    } else {
      x = ((abs / heuristic_abs_min) * heuristic_factor) * (x / abs);
      y = ((abs / heuristic_abs_min) * heuristic_factor) * (y / abs);
    }
  }

  cv_bridge::CvImage cvImagePot;
  cvImagePot.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  cvImagePot.image = potentialField;
  cv_bridge::CvImage cvImageVec;
  cvImageVec.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
  cvImageVec.image = vectorField;
  imagePublisherPot.publish(cvImagePot.toImageMsg());
  imagePublisherVec.publish(cvImageVec.toImageMsg());
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", ros::this_node::getName().c_str());
  // Init ROS
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");

  string amiroOdomListenerTopic;
  string rosPublisherTopicPot, rosPublisherTopicVec;
  node.param<string>("amiro_odom_listener_topic", amiroOdomListenerTopic, "/amiro1/odom");
  node.param<string>("potentialfield_publisher_topic", rosPublisherTopicPot, "/potentialfield/amiro1");
  node.param<string>("vectorfield_publisher_topic", rosPublisherTopicVec, "/vectorfield/amiro1");
  node.param<double>("meter_per_pixel", meterPerPixel, 0.003);
  node.param<float>("heuristic_abs_min", heuristic_abs_min, 0.2);
  node.param<float>("heuristic_factor", heuristic_factor, 0.2);
  node.param<int>("image_width", imageWidth, 1000);
  node.param<int>("image_height", imageHeight, 1000);
  node.param<int>("minimum_pose_difference_pixel", minPoseDiff_pixel, 10);
  ROS_INFO("[%s] image_listener_topic: %s", ros::this_node::getName().c_str(), amiroOdomListenerTopic.c_str());
  ROS_INFO("[%s] potentialfield_publisher_topic: %s", ros::this_node::getName().c_str(), rosPublisherTopicPot.c_str());
  ROS_INFO("[%s] vectorfield_publisher_topic: %s", ros::this_node::getName().c_str(), rosPublisherTopicVec.c_str());
  ROS_INFO("[%s] meter_per_pixel: %f", ros::this_node::getName().c_str(), meterPerPixel);
  ROS_INFO("[%s] image_width: %d", ros::this_node::getName().c_str(), imageWidth);
  ROS_INFO("[%s] image_height: %d", ros::this_node::getName().c_str(), imageHeight);
  ROS_INFO("[%s] minimum_pose_difference_pixel: %d", ros::this_node::getName().c_str(), minPoseDiff_pixel);

  image_transport::ImageTransport imageTransport(node);
  imagePublisherPot = imageTransport.advertise(rosPublisherTopicPot, 1, true);
  imagePublisherVec = imageTransport.advertise(rosPublisherTopicVec, 1, true);
  ros::Subscriber odom_sub = node.subscribe(amiroOdomListenerTopic, 1, &process);

  ros::spin();
  return 0;

}
