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


using namespace std;


// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisher;

// cv::Mat should be thread safe
const std::size_t numFields = 2;
static std::vector<cv::Mat> fields(numFields);
static std::vector<bool> dataArrived(numFields,false);

void process1(const sensor_msgs::ImageConstPtr& msg) {
  const std::size_t id = 0;
  dataArrived.at(id) = true;
  cv_bridge::toCvShare(msg, msg->encoding)->image.copyTo(fields.at(id));
  ROS_DEBUG_STREAM("process2, size (x,y): " << fields.at(id).cols << " "  << fields.at(id).rows);
}

void process2(const sensor_msgs::ImageConstPtr& msg) {
  const std::size_t id = 1;
  dataArrived.at(id) = true;
  cv_bridge::toCvShare(msg, msg->encoding)->image.copyTo(fields.at(id));
  ROS_DEBUG_STREAM("process1, size (x,y): " << fields.at(id).cols << " "  << fields.at(id).rows);
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

  {
    cv::Mat dummy = cv::Mat(cv::Size(fieldWidth,fieldHeight), CV_32FC2);
    for (auto it = fields.begin(); it < fields.end(); ++it) {
      dummy.copyTo(*it);
    }
  }
  cv_bridge::CvImage cvImage;
  cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
  cv::Mat vectorfield_merged = cv::Mat(cv::Size(fieldWidth,fieldHeight), CV_32FC2);

  image_transport::ImageTransport fusedTransport(node);
  image_transport::ImageTransport field1Transport(node);
  image_transport::ImageTransport field2Transport(node);
  imagePublisher = fusedTransport.advertise(vectorfieldPublisherTopic, 1, true);
  image_transport::Subscriber field1_sub = field1Transport.subscribe(vectorfield1ListenerTopic, 1, &process1);
  image_transport::Subscriber field2_sub = field2Transport.subscribe(vectorfield2ListenerTopic, 1, &process2);

  bool burnIn = true;
  ros::Rate rate(1);
  while(ros::ok()) {
    if (!burnIn) {
      if (dataArrived.at(0) || dataArrived.at(1)) {
        dataArrived.at(0) = false;
        dataArrived.at(1) = false;
        ROS_INFO_STREAM(ros::this_node::getName() << " STATE: Do fusion");
        // Fuse
        if (fields.at(0).cols != fieldWidth || fields.at(0).rows != fieldHeight ||
            fields.at(1).cols != fieldWidth || fields.at(1).rows != fieldHeight) {
          ROS_WARN_STREAM(ros::this_node::getName() << " insufficient field size");
          continue;
        }
        vectorfield_merged = fields.at(0) + fields.at(1);
        // Publish
        cvImage.image = vectorfield_merged;
        imagePublisher.publish(cvImage.toImageMsg());
      } else {
        ROS_DEBUG_STREAM(ros::this_node::getName() << " STATE: Skip fusion");
      }
    } else {
      ROS_INFO_STREAM(ros::this_node::getName() << " STATE: Burn in");
      if (dataArrived.at(0) && dataArrived.at(1)) {
        burnIn = false;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
