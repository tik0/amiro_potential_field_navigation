// ============================================================================
// Name        : potentialfield_merger_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve all potentialfields and merger them.
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

// OpenCV
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/subscriber_filter.h>


using namespace std;

int amiroNr;

// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisher;

cv::Mat potentialfield;

// program name
const string programName = "potentialfield_merger_node";

void processImagePotentialfield(const sensor_msgs::ImageConstPtr& msg) {
  potentialfield = cv_bridge::toCvShare(msg, msg->encoding)->image;
}

void processMergeAllPotentialfieldsSynced(const sensor_msgs::ImageConstPtr& pt1, const sensor_msgs::ImageConstPtr& pt2) {
  cv::Mat potentialfield_merged(potentialfield.size(), CV_32FC2);
  potentialfield_merged = potentialfield + cv_bridge::toCvShare(pt1, pt1->encoding)->image + cv_bridge::toCvShare(pt2, pt2->encoding)->image;

  cv_bridge::CvImage cvImage;
  cvImage.encoding = sensor_msgs::image_encodings::TYPE_32FC2;
  cvImage.image = potentialfield_merged;
  imagePublisher.publish(cvImage.toImageMsg());
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());
  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  string potentialfieldPublisherTopic;
  string imagePotentialFieldListenerTopic;
  node.param<string>("image_potentialfield_listener_topic", imagePotentialFieldListenerTopic, "/image/potentialfield");
  node.param<string>("potentialfield_publisher_topic", potentialfieldPublisherTopic, "/merged/potentialfield");
  node.param<int>("amiro_number", amiroNr, 2);
  ROS_INFO("[%s] image_potentialfield_listener_topic: %s", programName.c_str(), imagePotentialFieldListenerTopic.c_str());
  ROS_INFO("[%s] potentialfield_publisher_topic: %s", programName.c_str(), potentialfieldPublisherTopic.c_str());
  ROS_INFO("[%s] amiro_number: %d", programName.c_str(), amiroNr);

  string amiro1_pf_listener_topic;
  string amiro2_pf_listener_topic;
  node.param<string>("amiro1_pf_listener_topic", amiro1_pf_listener_topic, "/amiro1/potentialfield");
  node.param<string>("amiro2_pf_listener_topic", amiro2_pf_listener_topic, "/amiro2/potentialfield");
  ROS_INFO("[%s] amiro1_pf_listener_topic: %s", programName.c_str(), amiro1_pf_listener_topic.c_str());
  ROS_INFO("[%s] amiro2_pf_listener_topic: %s", programName.c_str(), amiro2_pf_listener_topic.c_str());

  image_transport::ImageTransport imageTransport(node);
  imagePublisher = imageTransport.advertise(potentialfieldPublisherTopic, 1);
  image_transport::Subscriber image_sub = imageTransport.subscribe(imagePotentialFieldListenerTopic, 1, &processImagePotentialfield);

  image_transport::SubscriberFilter amiro_pf_sub1(imageTransport, amiro1_pf_listener_topic, 1);
  image_transport::SubscriberFilter amiro_pf_sub2(imageTransport, amiro2_pf_listener_topic, 1);
//
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amiro_pf_sub1, amiro_pf_sub2);
  sync.registerCallback(boost::bind(&processMergeAllPotentialfieldsSynced, _1, _2));

  ros::spin();
  return 0;
}
