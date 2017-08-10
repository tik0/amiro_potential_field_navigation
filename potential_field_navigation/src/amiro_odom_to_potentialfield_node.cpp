// ============================================================================
// Name        : amiro_odom_to_potentialfield_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve a amiro odometry and creates a potentialfield.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
// ROS - OpenCV_ Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "potentialfield_image_converter.hpp"

#include <omp.h>


using namespace std;

#define C 128
//params
double meterPerPixel;
double potentialRadius;
int imageWidth;
int imageHeight;

// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisher;


// program name
const string programName = "amiro_odom_to_potentialfield_node";

void process(const nav_msgs::OdometryConstPtr &odom) {
  cv::Mat potentialField(imageHeight, imageWidth, CV_32FC2);
  cv::Point2i pose2d((int) (odom->pose.pose.position.x / meterPerPixel + imageWidth/2), (int) (odom->pose.pose.position.y / meterPerPixel + imageHeight/2));
  double potentialRadiusPixel = potentialRadius / meterPerPixel;

#pragma omp parallel for
  for (int y = 0; y < potentialField.rows; y++) {
    for (int x = 0; x < potentialField.cols; x++) {
      cv::Point2i curpx(x, y);
      cv::Point diff = curpx - pose2d;
      double distance = diff.x * diff.x + diff.y * diff.y;
      if (distance < potentialRadiusPixel) {
        cout << "debug" << curpx << endl;
        potentialField.at<cv::Vec2f>(y, x)[0] = diff.x * C;
        potentialField.at<cv::Vec2f>(y, x)[1] = diff.y * C;
      }
    }
  }

//  cv::imshow("bla", potentialfield_to_rgb_cv_mat(potentialField));
//  cv::waitKey(0);

  cv_bridge::CvImage cvImage;
  cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
  cvImage.image = potentialField;
  imagePublisher.publish(cvImage.toImageMsg());
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());
  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  string amiroOdomListenerScope;
  string imagePublisherScope;
  node.param<string>("amiro_odom_listener_scope", amiroOdomListenerScope, "/amiro1/odom");
  node.param<string>("potentialfield_publisher_topic", imagePublisherScope, "/amiro1/potentialfield");
  node.param<double>("meter_per_pixel", meterPerPixel, 0.003);
  node.param<double>("potential_radius", potentialRadius, 0.2);
  node.param<int>("image_width", imageWidth, 1000);
  node.param<int>("image_height", imageHeight, 1000);
  ROS_INFO("[%s] image_listener_topic: %s", programName.c_str(), amiroOdomListenerScope.c_str());
  ROS_INFO("[%s] potentialfield_publisher_topic: %s", programName.c_str(), imagePublisherScope.c_str());
  ROS_INFO("[%s] meter_per_pixel: %f", programName.c_str(), meterPerPixel);
  ROS_INFO("[%s] potential_radius: %f", programName.c_str(), potentialRadius);
  ROS_INFO("[%s] image_width: %d", programName.c_str(), imageWidth);
  ROS_INFO("[%s] image_height: %d", programName.c_str(), imageHeight);

  image_transport::ImageTransport imageTransport(node);
  imagePublisher = imageTransport.advertise(imagePublisherScope, 1);
  ros::Subscriber odom_sub = node.subscribe(amiroOdomListenerScope, 1, &process);

  ros::spin();
  return 0;

}
