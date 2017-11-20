// ============================================================================
// Name        : image_to_vectorfield_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Receive a image and creates a vectorfield.
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
image_transport::Publisher imagePublisherPot, imagePublisherVec;
static int image_flip_code;

// Sanity check for vectorfield size
static int desired_vectorfield_width, desired_vectorfield_height;

// Heuristic values
static float heuristic_factor = 1.0f;
static float heuristic_abs_min = 1.0f;
static int heuristic_apply;

void process(const sensor_msgs::ImageConstPtr &msg) {
  cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
//  rot90(image, RotateFlags::ROTATE_90_COUNTERCLOCKWISE);
  cv::Mat bgr[3];
  cv::split(image,bgr);
  cv::Mat potentialFieldRed(image.size(), CV_32FC1, cv::Scalar(0.0f)), potentialFieldBlue(image.size(), CV_32FC1, cv::Scalar(0.0f));
  cv::Mat vectorFieldRed(image.size(), CV_32FC2, cv::Scalar(0.0f, 0.0f)), vectorFieldBlue(image.size(), CV_32FC2, cv::Scalar(0.0f, 0.0f));
  cv::Mat vectorField(image.size(), CV_32FC2, cv::Scalar(0.0f, 0.0f));

  // Get list of red and blue pixels
  std::vector<cv::Point> redPixel, bluePixel;
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      if (bgr[0].at<uchar>(y, x) > 0) {
        bluePixel.emplace_back(cv::Point(x, y));
      }
      if (bgr[2].at<uchar>(y, x) > 0) {
        redPixel.emplace_back(cv::Point(x, y));
      }
    }
  }

  if (redPixel.empty() && bluePixel.empty()) {
    ROS_WARN_STREAM(ros::this_node::getName() << " No charge present, skip image");
  }

  // Calculate normalized potential field for the blue channel
  // TODO Differentiate between Charge and Current, because charge as no sqrt in the denominator
  for (auto it = bluePixel.begin(); it < bluePixel.end(); ++it) {
    const uchar value = bgr[0].at<uchar>(it->y, it->x);
#pragma omp parallel for
    for (int y = 0; y < bgr[0].rows; y++) {
      for (int x= 0; x < bgr[0].cols; x++) {
        if (it->y == y && it->x == x) {
          continue;
        }
        // We assume an attracting charge/current
        potentialFieldBlue.at<float>(y, x) += - (value / 255.0f) / sqrt(pow(y-it->y,2)+pow(x-it->x,2));
      }
    }
  }
  // Calculate normalized potential field for the red channel
  for (auto it = redPixel.begin(); it < redPixel.end(); ++it) {
    const uchar value = bgr[2].at<uchar>(it->y, it->x);
#pragma omp parallel for
    for (int y = 0; y < bgr[2].rows; y++) {
      for (int x= 0; x < bgr[2].cols; x++) {
        if (it->y == y && it->x == x) {
          continue;
        }
        // We assume a repelling charge/current
        potentialFieldRed.at<float>(y, x) += (value / 255.0f) / sqrt(pow(y-it->y,2)+pow(x-it->x,2));
      }
    }
  }

  if (msg->header.frame_id.compare("charge") == 0) {
    ROS_INFO("[%s] calculate image as charge", ros::this_node::getName().c_str());
    // Get vector field
    vectorFieldRed = potentialfield_to_vectorfield(potentialFieldRed, false);
    vectorFieldBlue = potentialfield_to_vectorfield(potentialFieldBlue, false);
  } else if (msg->header.frame_id.compare("current") == 0) {
    ROS_INFO("[%s] calculate image as current", ros::this_node::getName().c_str());
    // Get vector field
    vectorFieldRed = potentialfield_to_vectorfield(potentialFieldRed, true);
    vectorFieldBlue = potentialfield_to_vectorfield(potentialFieldBlue, true);
  } else {
    ROS_WARN_STREAM(ros::this_node::getName() << " unknown frame_id: " << msg->header.frame_id);
    return;
  }

  // Apply heuristics (S.t. calculate the motor schema)
  if (heuristic_apply) {
    // Schema for currents, s.t. rotate around a charge with velocities which become
    // weaker with closer distance to the current
    if (msg->header.frame_id.compare("current") == 0) {
      cv::add(vectorFieldRed, vectorFieldBlue, vectorField);
#pragma omp parallel for
      for (int idy = 0; idy < vectorField.rows; idy++) {
        for (int idx = 0; idx < vectorField.cols; idx++) {
          float &x = vectorField.at<cv::Vec2f>(idy, idx)[0];
          float &y = vectorField.at<cv::Vec2f>(idy, idx)[1];
          float abs = cv::norm(vectorField.at<cv::Vec2f>(idy, idx));

          // Remove value if on charge
          if (bgr[0].at<uchar>(idy, idx) > 0 || bgr[2].at<uchar>(idy, idx) > 0) {
            x = 0.0f;
            y = 0.0f;
          } else if (abs < heuristic_abs_min) { // Keep vector which are far away constant
            const float factor = heuristic_factor;
            x = factor * x / abs;
            y = factor * y / abs;
          } else {  // Degenerate vector as closer they are to the charge
            const float factor = heuristic_factor * heuristic_abs_min / abs;
            x = factor * x / abs;
            y = factor * y / abs;
          }
        }
      }
    } else { // if (msg->header.frame_id.compare("charge") == 0)
      // Schema for attracting charge with velocities which become
      // weaker with closer distance to the charge
      if (!bluePixel.empty()) {
#pragma omp parallel for
        for (int idy = 0; idy < vectorFieldBlue.rows; idy++) {
          for (int idx = 0; idx < vectorFieldBlue.cols; idx++) {
            float &x = vectorFieldBlue.at<cv::Vec2f>(idy, idx)[0];
            float &y = vectorFieldBlue.at<cv::Vec2f>(idy, idx)[1];
            float abs = cv::norm(vectorFieldBlue.at<cv::Vec2f>(idy, idx));

            // Remove value if on charge
            if (bgr[0].at<uchar>(idy, idx) > 0) {
              x = 0.0f;
              y = 0.0f;
            } else if (abs < heuristic_abs_min) { // Keep vector which are far away constant
              const float factor = heuristic_factor;
              x = factor * x / abs;
              y = factor * y / abs;
            } else {  // Degenerate vector as closer they are to the charge
              const float factor = heuristic_factor * heuristic_abs_min / abs;
              x = factor * x / abs;
              y = factor * y / abs;
            }
          }
        }
      }
      // Schema for repelling charge with velocities which become
      // stronger with closer distance to the charge
      if (!redPixel.empty()) {
#pragma omp parallel for
        for (int idy = 0; idy < vectorFieldRed.rows; idy++) {
          for (int idx = 0; idx < vectorFieldRed.cols; idx++) {
            float &x = vectorFieldRed.at<cv::Vec2f>(idy, idx)[0];
            float &y = vectorFieldRed.at<cv::Vec2f>(idy, idx)[1];
            float abs = cv::norm(vectorFieldRed.at<cv::Vec2f>(idy, idx));

            // Normalize value if on charge
            if (bgr[2].at<uchar>(idy, idx) > 0) {
              const float factor = heuristic_factor;
              x = factor * x / abs;
              y = factor * y / abs;
            } else if (abs < heuristic_abs_min) { // Keep vector which are far away constant
              x = 0.0f;
              y = 0.0f;
            } else {  // Straighten vector as closer they are to the charge
              const float factor = heuristic_factor * (abs - heuristic_abs_min) / abs;
              x = factor * x / abs;
              y = factor * y / abs;
            }
          }
        }
      }

/*      for (int idy = 0; idy < vectorFieldRed.rows; idy++) {
        for (int idx = 0; idx < vectorFieldRed.cols; idx++) {
          vectorField.at<cv::Vec2f>(idy, idx)[0] = vectorFieldRed.at<cv::Vec2f>(idy, idx)[0] + vectorFieldBlue.at<cv::Vec2f>(idy, idx)[0];
          vectorField.at<cv::Vec2f>(idy, idx)[1] = vectorFieldRed.at<cv::Vec2f>(idy, idx)[1] + vectorFieldBlue.at<cv::Vec2f>(idy, idx)[1];
          ROS_WARN_STREAM("r " << vectorFieldRed.at<cv::Vec2f>(idy, idx)[0] << " " << vectorFieldRed.at<cv::Vec2f>(idy, idx)[1]);
          ROS_WARN_STREAM("b " << vectorFieldBlue.at<cv::Vec2f>(idy, idx)[0] << " " << vectorFieldBlue.at<cv::Vec2f>(idy, idx)[1]);
        }
      }*/

      // Subsum the vector fields of the charges
      cv::add(vectorFieldBlue, vectorFieldRed, vectorField);
    }
  }

  // Sanity check for vectorfield size
  if (desired_vectorfield_width > 0 && desired_vectorfield_height > 0) {
    ROS_INFO_STREAM("Resizing desired ...");
    if (vectorField.rows != desired_vectorfield_height || vectorField.cols != desired_vectorfield_width) {
      ROS_INFO_STREAM("resize vectorfield to desired size: (" << desired_vectorfield_height << ", " << desired_vectorfield_width << ")");
      cv::resize(vectorField, vectorField, cv::Size(desired_vectorfield_width, desired_vectorfield_height), 0, 0, cv::INTER_LINEAR);
    } else {
      ROS_INFO_STREAM("no resizing necessary");
    }
  }

  // Send the data
  cv_bridge::CvImage cvImagePot;
  cvImagePot.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  cvImagePot.image = potentialFieldBlue + potentialFieldBlue;
//  rot90(cvImagePot.image, RotateFlags::ROTATE_90_CLOCKWISE);
  cv_bridge::CvImage cvImageVec;
  cvImageVec.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
  cvImageVec.image = vectorField;
  imagePublisherPot.publish(cvImagePot.toImageMsg());
  ROS_INFO("[%s] publish potentialfield.", ros::this_node::getName().c_str());
  imagePublisherVec.publish(cvImageVec.toImageMsg());
  ROS_INFO("[%s] publish vectorfield.", ros::this_node::getName().c_str());

}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", ros::this_node::getName().c_str());
  // Init ROS
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");
  node.param<string>("image_listener_topic", rosListenerTopic, "/image");
  node.param<string>("potentialfield_publisher_topic", rosPublisherTopicPot, "/image/potentialfield");
  node.param<string>("vectorfield_publisher_topic", rosPublisherTopicVec, "/image/vectorfield");
  node.param<int>("heuristic_apply", heuristic_apply, 1);
  node.param<float>("heuristic_factor", heuristic_factor, 1.0);
  node.param<float>("heuristic_abs_min", heuristic_abs_min, 1.0);
  node.param<int>("image_flip_code", image_flip_code, 0);
  node.param<int>("desired_vectorfield_width", desired_vectorfield_width, 0);
  node.param<int>("desired_vectorfield_height", desired_vectorfield_height, 0);

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
