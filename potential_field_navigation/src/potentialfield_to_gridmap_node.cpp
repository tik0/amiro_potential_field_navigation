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

#include <grid_map_msgs/GridMap.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <omp.h>

#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

using namespace std;

#define meterPerPixel 0.003

// Ros Topics
string rosListenerTopic;
string rosPublisherTopic;

ros::Publisher gridMapPublisher;

// program name
const string programName = "potentialfield_to_gridmap_node";


void process(const sensor_msgs::ImageConstPtr &msg) {
  grid_map_msgs::GridMap gridmap_msg;

  grid_map::GridMap gridmap;
  grid_map::GridMapRosConverter::initializeFromImage(*msg, meterPerPixel, gridmap);

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
  node.param<string>("potentialfield_listener_topic", rosListenerTopic, "/potentialfield");
  ROS_INFO("potentialfield_listener_topic: %s", rosListenerTopic.c_str());
  node.param<string>("gridmap_publisher_topic", rosPublisherTopic, "/gridmap");
  ROS_INFO("gridmap_publisher_topic: %s", rosPublisherTopic.c_str());

  gridMapPublisher = node.advertise<grid_map_msgs::GridMap>(rosPublisherTopic, 1);

  image_transport::ImageTransport imageTransport(node);
  image_transport::Subscriber image_sub = imageTransport.subscribe(rosListenerTopic, 1, &process);


  ros::spin();
  return 0;

}
