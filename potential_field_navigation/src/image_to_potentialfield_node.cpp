// ============================================================================
// Name        : image_to_potentialfield_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve a image and creates a potentialfield.
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

#include "vectorfield_image_converter.hpp"

using namespace std;

// Ros Topics
string rosListenerTopic;
string rosPublisherTopicPot, rosPublisherTopicVec;

// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisherPot, imagePublisherVec;


// program name
const string programName = "image_to_potentialfield_node";

void process(const sensor_msgs::ImageConstPtr &msg) {
  boost::posix_time::ptime before = boost::posix_time::microsec_clock::local_time();
  cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
  cv::Mat potentialField(image.size(), CV_32FC2); // TODO Call it vector field
  cv::Mat vectorField(image.size(), CV_32FC2);
  cv::Mat vectorFieldX(image.size(), CV_32FC1);
  cv::Mat vectorFieldY(image.size(), CV_32FC1);

  cv::Mat potentialField2(image.size(), CV_32FC1);

  vector<cv::Point> blackPixel;
  vector<cv::Point> whitePixel;
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      if (image.at<uchar>(y, x) == 0) {
        blackPixel.emplace_back(cv::Point(x, y));
      } else {
        whitePixel.emplace_back(cv::Point(x, y));
      }
    }
  }

#pragma omp parallel for
  for (int y = 0; y < image.rows; y++) {
    for (int x= 0; x < image.cols; x++) {
      for (auto it = blackPixel.begin(); it < blackPixel.end(); ++it) {
        if (it->y == y && it->x == x) {
          continue;
        }
        potentialField2.at<float>(y, x) += 1.0 / sqrt(pow(y-it->y,2)+pow(x-it->x,2));
      }
    }
  }

  /// Update kernel size for a normalized box filter
  int kernel_size = 3;
  cv::Mat kernelX = cv::Mat::zeros( kernel_size, kernel_size, CV_32F );
  cv::Mat kernelY = cv::Mat::zeros( kernel_size, kernel_size, CV_32F );
  kernelY.at<float>(0,0) = -1;
  kernelY.at<float>(0,1) = -2;
  kernelY.at<float>(0,2) = -1;
  kernelY.at<float>(2,0) = 1;
  kernelY.at<float>(2,1) = 2;
  kernelY.at<float>(2,2) = 1;
  kernelX.at<float>(0,0) = -1;
  kernelX.at<float>(1,0) = -2;
  kernelX.at<float>(2,0) = -1;
  kernelX.at<float>(0,2) = 1;
  kernelX.at<float>(1,2) = 2;
  kernelX.at<float>(2,2) = 1;

  /// Apply filter
  cv::Point anchor = cv::Point( -1, -1 );
  double delta = 0;
  cv::filter2D(potentialField2, vectorFieldX, -1 /*ddepth*/ , kernelX, anchor, delta, cv::BORDER_REPLICATE );
  cv::filter2D(potentialField2, vectorFieldY, -1 /*ddepth*/ , kernelY, anchor, delta, cv::BORDER_REPLICATE );

  // merge

  cv::Mat vectorFieldTmp[2] = {vectorFieldX, vectorFieldY};
  cv::merge(vectorFieldTmp,2,vectorField);

//  cv::imshow("out", vectorfield_to_bgr_cv_mat(vectorField));
  cv::imshow("out_potentialField_HSV", vectorfield_to_bgr_cv_mat(vectorField));
//  cv::imshow("out_potentialField", potentialField2);
  cv::waitKey(0);
  cv_bridge::CvImage cvImagePot;
  cvImagePot.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  cvImagePot.image = potentialField2;
  cv_bridge::CvImage cvImageVec;
  cvImageVec.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
  cvImageVec.image = vectorField;
  imagePublisherPot.publish(cvImagePot.toImageMsg());
  imagePublisherVec.publish(cvImageVec.toImageMsg());
  return;

//  vector<cv::Point> blackPixel;
//  vector<cv::Point> whitePixel;
//  for (int y = 0; y < image.rows; y++) {
//    for (int x = 0; x < image.cols; x++) {
//      if (image.at<uchar>(y, x) == 0)
//        blackPixel.emplace_back(cv::Point(y, x));
//      else
//        whitePixel.emplace_back(cv::Point(y, x));
//    }
//  }
//
//#pragma omp parallel for
//  for (int i = 0; i < (signed) whitePixel.size(); i++) {
//    cv::Point wPoint = whitePixel[i];
//    cv::Point potential;
//    for (cv::Point bPoint : blackPixel) {
//      potential += bPoint - wPoint;
////      double angle = atan2(wPoint.y - bPoint.y, wPoint.x - bPoint.x);
//    }
//    potentialField.at<cv::Vec2f>(wPoint.y, wPoint.x)[0] = potential.x / (float) blackPixel.size();
//    potentialField.at<cv::Vec2f>(wPoint.y, wPoint.x)[1] = potential.y / (float) blackPixel.size();
//  }
//
//  cv::imshow("out", vectorfield_to_bgr_cv_mat(potentialField));
//  cv::waitKey(1);
//  cv_bridge::CvImage cvImage;
//  cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
//  cvImage.image = potentialField;
//  boost::posix_time::ptime after = boost::posix_time::microsec_clock::local_time();
//  ROS_INFO("[%s] process image to potentialfield in %i ms", programName.c_str(), (int) (after - before).total_milliseconds());
//  imagePublisher.publish(cvImage.toImageMsg());
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());
  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");
  node.param<string>("image_listener_topic", rosListenerTopic, "/image");
  node.param<string>("potentialfield_publisher_topic", rosPublisherTopicPot, "/image/potentialfield");
  node.param<string>("vectorfield_publisher_topic", rosPublisherTopicVec, "/image/vectorfield");
  ROS_INFO("[%s] image_listener_topic: %s", programName.c_str(), rosListenerTopic.c_str());
  ROS_INFO("[%s] vectorfield_publisher_topic: %s", programName.c_str(), rosPublisherTopicVec.c_str());
  ROS_INFO("[%s] potentialfield_publisher_topic: %s", programName.c_str(), rosPublisherTopicPot.c_str());

  image_transport::ImageTransport imageTransport(node);
  image_transport::Subscriber sub = imageTransport.subscribe(rosListenerTopic, 1, process);
  imagePublisherPot = imageTransport.advertise(rosPublisherTopicPot, 1, true);
  imagePublisherVec = imageTransport.advertise(rosPublisherTopicVec, 1, true);


  ros::spin();
  return 0;

}
