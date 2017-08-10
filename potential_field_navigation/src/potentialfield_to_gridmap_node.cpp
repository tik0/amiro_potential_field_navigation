// ============================================================================
// Name        : potentialfield_to_gridmap_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve a potentialfield and creates a grid_map.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
// ROS - OpenCV_ Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Grid Map
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

//OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "potentialfield_image_converter.hpp"

#include <omp.h>

using namespace std;

// Ros Topics
string rosListenerTopic;
string rosPublisherTopic;

ros::Publisher gridMapPublisher;

// program name
const string programName = "potentialfield_to_gridmap_node";

//params
double meterPerPixel;

void process(const sensor_msgs::ImageConstPtr &msg) {
//  cv::Mat potentialfield = (cv_bridge::toCvShare(msg, msg->encoding)->image;


  cv::Mat rgbCv = potentialfield_to_rgb_cv_mat(cv_bridge::toCvShare(msg, msg->encoding)->image);
  cv_bridge::CvImage cvImage;
  cvImage.header = msg->header;
  cvImage.encoding = sensor_msgs::image_encodings::RGB8;
  cvImage.image = rgbCv;
  sensor_msgs::ImagePtr rgbImage = cvImage.toImageMsg();
  // Nice idea but only works with normal RGB Images
  grid_map::GridMap gridmap;
  grid_map::GridMapRosConverter::initializeFromImage(*rgbImage, meterPerPixel, gridmap);
  grid_map::GridMapRosConverter::addLayerFromImage(*rgbImage, "elevation", gridmap);
  grid_map::GridMapRosConverter::addColorLayerFromImage(*rgbImage, "color", gridmap);
  grid_map_msgs::GridMap gridmap_msg;
  grid_map::GridMapRosConverter::toMessage(gridmap, gridmap_msg);

  gridmap_msg.info.header.frame_id = "world";
  ROS_INFO("[%s] publish gridmap", programName.c_str());
  gridMapPublisher.publish(gridmap_msg);
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());
  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");
  node.param<string>("potentialfield_listener_topic", rosListenerTopic, "/image/potentialfield");
  node.param<string>("gridmap_publisher_topic", rosPublisherTopic, "/gridmap");
  node.param<double>("meter_per_pixel", meterPerPixel, 0.003);
  ROS_INFO("[%s] potentialfield_listener_topic: %s", programName.c_str(), rosListenerTopic.c_str());
  ROS_INFO("[%s] gridmap_publisher_topic: %s", programName.c_str(), rosPublisherTopic.c_str());
  ROS_INFO("[%s] meter_per_pixel: %f", programName.c_str(), meterPerPixel);


  gridMapPublisher = node.advertise<grid_map_msgs::GridMap>(rosPublisherTopic, 1, true);

  image_transport::ImageTransport imageTransport(node);
  image_transport::Subscriber image_sub = imageTransport.subscribe(rosListenerTopic, 1, &process);


  ros::spin();
  return 0;

}
