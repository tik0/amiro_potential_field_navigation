// ============================================================================
// Name        : image_to_vectorfield_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkrothals@cit-ec.uni-bielefeld.de>
// Description : Recieve a image and creates a vectorfield.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// ROS - OpenCV_ Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

#include <omp.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "potential_field_utils.hpp"

using namespace std;

// Ros Topics
string rosListenerTopic;
string rosPublisherTopicPot, rosPublisherTopicVec;

// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisherPot, imagePublisherVec;

// Heuristic values
float heuristic_factor = 1.0f;
float heuristic_abs_min = 1.0f;

void process(const sensor_msgs::ImageConstPtr &msg) {
  cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
  cv::Mat potentialField(image.size(), CV_32FC1);

  // Get list of black pixels
  std::vector<cv::Point> blackPixel;
  std::vector<cv::Point> whitePixel;
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      if (image.at<uchar>(y, x) == 0) {
        blackPixel.emplace_back(cv::Point(x, y));
      } else {
        whitePixel.emplace_back(cv::Point(x, y));
      }
    }
  }

  // Calculate normalized vector field
#pragma omp parallel for
  for (int y = 0; y < image.rows; y++) {
    for (int x= 0; x < image.cols; x++) {
      for (auto it = blackPixel.begin(); it < blackPixel.end(); ++it) {
        if (it->y == y && it->x == x) {
          continue;
        }
        potentialField.at<float>(y, x) += 1.0 / sqrt(pow(y-it->y,2)+pow(x-it->x,2));
      }
    }
  }

  // Get vector field
  cv::Mat vectorField = potentialfield_to_vectorfield(potentialField);

  // Apply heuristics
  for (int idy = 0; idy < vectorField.rows; idy++) {
    for (int idx = 0; idx < vectorField.cols; idx++) {
      float &x = vectorField.at<cv::Vec2f>(idy, idx)[0];
      float &y = vectorField.at<cv::Vec2f>(idy, idx)[1];
      const float abs = sqrt(x*x + y*y);

      // Remove value if on charge
      if (image.at<uchar>(idy, idx) == 0) {
        x = 0.0;
        y = 0.0;
      }
      // Keep vector which are far away constant
      else if (abs < heuristic_abs_min) {
        const float factor = heuristic_factor;
        x = factor * x / abs;
        y = factor * y / abs;
      }
      // Degenerate vector as closer they are to the charge
      else { /*(abs < abs_min)*/
        const float factor = heuristic_factor * heuristic_abs_min / abs;
        x = factor * x / abs;
        y = factor * y / abs;
      }
    }
  }

//  cv::imshow("out", vectorfield_to_bgr_cv_mat(vectorField));
//  cv::imshow("out_potentialField_HSV", vectorfield_to_bgr_cv_mat(vectorField));
//  cv::imshow("out_potentialField", potentialField);
//  cv::waitKey(0);
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
  node.param<string>("image_listener_topic", rosListenerTopic, "/image");
  node.param<string>("potentialfield_publisher_topic", rosPublisherTopicPot, "/image/potentialfield");
  node.param<string>("vectorfield_publisher_topic", rosPublisherTopicVec, "/image/vectorfield");
  node.param<float>("heuristic_factor", heuristic_factor, 1.0);
  node.param<float>("heuristic_abs_min", heuristic_abs_min, 1.0);
  ROS_INFO("[%s] image_listener_topic: %s", ros::this_node::getName().c_str(), rosListenerTopic.c_str());
  ROS_INFO("[%s] vectorfield_publisher_topic: %s", ros::this_node::getName().c_str(), rosPublisherTopicVec.c_str());
  ROS_INFO("[%s] potentialfield_publisher_topic: %s", ros::this_node::getName().c_str(), rosPublisherTopicPot.c_str());

  image_transport::ImageTransport imageTransport(node);
  image_transport::Subscriber sub = imageTransport.subscribe(rosListenerTopic, 1, process);
  imagePublisherPot = imageTransport.advertise(rosPublisherTopicPot, 1, true);
  imagePublisherVec = imageTransport.advertise(rosPublisherTopicVec, 1, true);

  ros::spin();
  return 0;

}
