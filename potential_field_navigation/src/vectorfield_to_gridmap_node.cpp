// ============================================================================
// Name        : vectorfield_to_gridmap_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Recieve a vectorfield and creates a grid_map.
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
#include <opencv2/highgui/highgui.hpp>

#include "potential_field_utils.hpp"

using namespace std;

// Ros Topics
string rosListenerTopic;
string rosPublisherTopic;

ros::Publisher gridMapPublisher;

//params
double meterPerPixel;
std::string frame_id;

void process(const sensor_msgs::ImageConstPtr &msg) {
  cv::Mat vectorField = cv_bridge::toCvShare(msg, msg->encoding)->image;
  cv::Mat hsv = vectorfield_to_hsv(vectorField, ros::this_node::getName());
  cv::Mat rgb = hsv_to_rgb(hsv);

  // Get the color
  cv_bridge::CvImage cvImage;
  cvImage.header = msg->header;
  cvImage.encoding = sensor_msgs::image_encodings::RGB8;
  cvImage.image = rgb;
  sensor_msgs::ImagePtr rgbImage = cvImage.toImageMsg();

  // Get the height
  cv::Mat mono = hsv_to_gray(hsv);
  cv_bridge::CvImage cvImageMono;
  cvImageMono.header = msg->header;
  cvImageMono.encoding = sensor_msgs::image_encodings::MONO8;
  cvImageMono.image = mono;
  sensor_msgs::ImagePtr monoImage = cvImageMono.toImageMsg();

  // Format the grid map
  grid_map::GridMap gridmap;
  grid_map::GridMapRosConverter::initializeFromImage(*rgbImage, meterPerPixel, gridmap);
  grid_map::GridMapRosConverter::addLayerFromImage(*monoImage, "elevation", gridmap);
  grid_map::GridMapRosConverter::addColorLayerFromImage(*rgbImage, "color", gridmap);
  grid_map_msgs::GridMap gridmap_msg;
  grid_map::GridMapRosConverter::toMessage(gridmap, gridmap_msg);

  gridmap_msg.info.header.frame_id = frame_id;
  ROS_INFO("[%s] publish gridmap", ros::this_node::getName().c_str());
  gridMapPublisher.publish(gridmap_msg);
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", ros::this_node::getName().c_str());
  // Init ROS
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");
  node.param<string>("vectorfield_listener_topic", rosListenerTopic, "/image/vectorfield");
  node.param<string>("gridmap_publisher_topic", rosPublisherTopic, "/gridmap");
  node.param<string>("frame_id", frame_id, "world");
  node.param<double>("meter_per_pixel", meterPerPixel, 0.003);
  ROS_INFO("[%s] potentialfield_listener_topic: %s", ros::this_node::getName().c_str(), rosListenerTopic.c_str());
  ROS_INFO("[%s] gridmap_publisher_topic: %s", ros::this_node::getName().c_str(), rosPublisherTopic.c_str());
  ROS_INFO("[%s] meter_per_pixel: %f", ros::this_node::getName().c_str(), meterPerPixel);


  gridMapPublisher = node.advertise<grid_map_msgs::GridMap>(rosPublisherTopic, 1, true);

  image_transport::ImageTransport imageTransport(node);
  image_transport::Subscriber image_sub = imageTransport.subscribe(rosListenerTopic, 1, &process);


  ros::spin();
  return 0;

}
