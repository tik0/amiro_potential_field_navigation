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
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <omp.h>

#include <boost/date_time/posix_time/posix_time.hpp>

using namespace std;

// Ros Topics
string rosListenerTopic;
string rosPublisherTopic;

// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisher;


// program name
const string programName = "image_to_potentialfield_node";

void process(const sensor_msgs::ImageConstPtr &msg) {
  boost::posix_time::ptime before = boost::posix_time::microsec_clock::local_time();
  cv::Mat image = cv_bridge::toCvShare(msg, msg->encoding)->image;
  cv::Mat potentialField(image.size(), CV_32FC2);

  vector<cv::Point> blackPixel;
  vector<cv::Point> whitePixel;
  for (int y = 0; y < image.rows; y++) {
    for (int x = 0; x < image.cols; x++) {
      if (image.at<uchar>(x, y) == 0)
        blackPixel.emplace_back(cv::Point(y, x));
      else
        whitePixel.emplace_back(cv::Point(y, x));
    }
  }

#pragma omp parallel for
  for (int i = 0; i < (signed) whitePixel.size(); i++) {
    cv::Point wPoint = whitePixel[i];
    cv::Point potential;
    for (cv::Point bPoint : blackPixel) {
      potential += bPoint - wPoint;
//      double angle = atan2(wPoint.y - bPoint.y, wPoint.x - bPoint.x);
    }
    potentialField.at<cv::Vec2f>(wPoint.y, wPoint.x)[0] = potential.x / (float) blackPixel.size();
    potentialField.at<cv::Vec2f>(wPoint.y, wPoint.x)[1] = potential.y / (float) blackPixel.size();
  }

//  cv::imshow("out", toMat(potentialField));
//  cv::waitKey(0);
  cv_bridge::CvImage cvImage;
  cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
  cvImage.image = potentialField;
  boost::posix_time::ptime after = boost::posix_time::microsec_clock::local_time();
  ROS_INFO("[%s] process image to potentialfield in %i ms", programName.c_str(), (int) (after - before).total_milliseconds());
  imagePublisher.publish(cvImage.toImageMsg());
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());
  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");
  node.param<string>("image_listener_topic", rosListenerTopic, "/image");
  node.param<string>("potentialfield_publisher_topic", rosPublisherTopic, "/image/potentialfield");
  ROS_INFO("[%s] image_listener_topic: %s", programName.c_str(), rosListenerTopic.c_str());
  ROS_INFO("[%s] potentialfield_publisher_topic: %s", programName.c_str(), rosPublisherTopic.c_str());

  image_transport::ImageTransport imageTransport(node);
  image_transport::Subscriber sub = imageTransport.subscribe(rosListenerTopic, 1, process);
  imagePublisher = imageTransport.advertise(rosPublisherTopic, 1, true);


  ros::spin();
  return 0;

}
