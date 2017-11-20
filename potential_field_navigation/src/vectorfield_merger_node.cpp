// ============================================================================
// Name        : vectorfield_merger_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Receive all vectorfields and merger them.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS - OpenCV_ Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>

#include "potential_field_utils.hpp"

using namespace std;


// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisher;

// cv::Mat should be thread safe
cv::Mat field1, field2;
static bool dataArrived1, dataArrived2;
static int normalize;
static float rate;

void process1(const sensor_msgs::ImageConstPtr& msg) {
  dataArrived1 = true;
  cv_bridge::toCvShare(msg, msg->encoding)->image.copyTo(field1);
  ROS_DEBUG_STREAM("process2, size (x,y): " << field1.cols << " "  << field1.rows);
}

void process2(const sensor_msgs::ImageConstPtr& msg) {
  dataArrived2 = true;
  cv_bridge::toCvShare(msg, msg->encoding)->image.copyTo(field2);
  ROS_DEBUG_STREAM("process1, size (x,y): " << field2.cols << " "  << field2.rows);
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", ros::this_node::getName().c_str());
  // Init ROS
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");

  string vectorfieldPublisherTopic;
  string vectorfield1ListenerTopic, vectorfield2ListenerTopic;
  int fieldWidth, fieldHeight;
  node.param<string>("field1_listener_topic", vectorfield1ListenerTopic, "/vectorfield/amiro1");
  node.param<string>("field2_listener_topic", vectorfield2ListenerTopic, "/vectorfield/amiro2");
  node.param<string>("vectorfield_publisher_topic", vectorfieldPublisherTopic, "/vectorfield/fused");
  node.param<int>("field_width", fieldWidth, 1000);
  node.param<int>("field_height", fieldHeight, 1000);
  node.param<int>("normalize", normalize, 0);
  node.param<float>("rate", rate, 1);

  // Initialize
  cv_bridge::CvImage cvImage;
  cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
  cv::Mat vectorfield_merged;
  {
    cv::Mat dummy = cv::Mat(cv::Size(fieldWidth,fieldHeight), CV_32FC2, cv::Scalar(0.0f));
    dummy.copyTo(field1);
    dummy.copyTo(field2);
    dummy.copyTo(vectorfield_merged);
  }

  image_transport::ImageTransport fusedTransport(node);
  image_transport::ImageTransport field1Transport(node);
  image_transport::ImageTransport field2Transport(node);
  imagePublisher = fusedTransport.advertise(vectorfieldPublisherTopic, 1, true);
  image_transport::Subscriber field1_sub = field1Transport.subscribe(vectorfield1ListenerTopic, 1, &process1);
  image_transport::Subscriber field2_sub = field2Transport.subscribe(vectorfield2ListenerTopic, 1, &process2);

  bool burnIn = true;
  ros::Rate r(rate);
  while(ros::ok()) {
    ros::spinOnce();
    if (!burnIn) {
      if (dataArrived1 || dataArrived2) {
        dataArrived1 = false;
        dataArrived2 = false;
        ROS_INFO_STREAM(ros::this_node::getName() << " STATE: Do fusion");
        // Fuse
        if (field1.cols != fieldWidth || field1.rows != fieldHeight ||
            field2.cols != fieldWidth || field2.rows != fieldHeight) {
          ROS_WARN_STREAM(ros::this_node::getName() << " insufficient field size: (field1.cols, field1.rows, field2.cols, field2.rows, fieldWidth, fieldHeight)=(" <<
                          field1.cols << ", " << field1.rows << ", " << field2.cols << ", " << field2.rows << ", " << fieldWidth << ", " << fieldHeight << ")");
          continue;
        }
        vectorfield_merged = field1 + field2;
        // Normalize to the greatest vector
        if (normalize) {
          calc_normalized_field(vectorfield_merged);
        }
        // Publish
        cvImage.image = vectorfield_merged;
        imagePublisher.publish(cvImage.toImageMsg());
      } else {
        ROS_DEBUG_STREAM(ros::this_node::getName() << " STATE: Skip fusion");
      }
    } else {
      ROS_INFO_STREAM(ros::this_node::getName() << " STATE: Burn in");
      if (dataArrived1 && dataArrived2) {
        burnIn = false;
      }
    }
    r.sleep();
  }
  return 0;
}
