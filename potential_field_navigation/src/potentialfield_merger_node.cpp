// ============================================================================
// Name        : potentialfield_merger_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve all potentialfields and merger them.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// ROS - OpenCV_ Bridge
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <omp.h>


using namespace std;


// ros::Publisher rosPublisher;
static image_transport::Publisher imagePublisher;

cv::Mat potentialfield;

// program name
const string programName = "potentialfield_merger_node";

void processImagePotentialfield(sensor_msgs::ImageConstPtr &msg) {
  potentialfield = cv_bridge::toCvShare(msg, msg->encoding)->image;
}

void processMergeAllPotentialfieldsSynced(sensor_msgs::ImageConstPtr &pt1, sensor_msgs::ImageConstPtr &pt2, sensor_msgs::ImageConstPtr &pt3, sensor_msgs::ImageConstPtr &pt4, sensor_msgs::ImageConstPtr &pt5, sensor_msgs::ImageConstPtr &pt6, sensor_msgs::ImageConstPtr &pt7, sensor_msgs::ImageConstPtr &pt8, sensor_msgs::ImageConstPtr &pt9) {
  // TODO
  // Merge all potentialfields.

}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", programName.c_str());
  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  string potentialfieldPublisherTopic;
  string imagePotentialFieldListenerTopic;
  int amiroNr;
  node.param<string>("image_potentialfield_listener_topic", imagePotentialFieldListenerTopic, "/image/potentialfield");
  node.param<string>("potentialfield_publisher_topic", potentialfieldPublisherTopic, "/amiro1/potentialfield");
  node.param<int>("amiro_number", amiroNr, 2);
  ROS_INFO("[%s] image_potentialfield_listener_topic: %s", programName.c_str(), imagePotentialFieldListenerTopic.c_str());
  ROS_INFO("[%s] potentialfield_publisher_topic: %s", programName.c_str(), potentialfieldPublisherTopic.c_str());
  ROS_INFO("[%s] amiro_number: %d", programName.c_str(), amiroNr);

  image_transport::ImageTransport imageTransport(node);
  imagePublisher = imageTransport.advertise(potentialfieldPublisherTopic, 1);
  ros::Subscriber odom_sub = node.subscribe(imagePotentialFieldListenerTopic, 1, &processImagePotentialfield);

  if (amiroNr < 2) {
    ROS_ERROR("[%s] amiro_number %d is lesser than 2. Please use atleast 2.", programName.c_str(), amiroNr);
  }
  if (amiroNr > 9) {
    ROS_WARN("[%s] amiro_number %d is greater than 9. Only the first 9th will be used.", programName.c_str(), amiroNr);
    amiroNr = 9;
  }

  message_filters::Subscriber<sensor_msgs::Image> amiro_pf_sub1;
  message_filters::Subscriber<sensor_msgs::Image> amiro_pf_sub2;
  message_filters::Subscriber<sensor_msgs::Image> amiro_pf_sub3;
  message_filters::Subscriber<sensor_msgs::Image> amiro_pf_sub4;
  message_filters::Subscriber<sensor_msgs::Image> amiro_pf_sub5;
  message_filters::Subscriber<sensor_msgs::Image> amiro_pf_sub6;
  message_filters::Subscriber<sensor_msgs::Image> amiro_pf_sub7;
  message_filters::Subscriber<sensor_msgs::Image> amiro_pf_sub8;
  message_filters::Subscriber<sensor_msgs::Image> amiro_pf_sub9;
  switch (amiroNr) {
    case 2: {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
      amiro_pf_sub1.subscribe(node, "/amiro1/potentialfield", 1);
      amiro_pf_sub2.subscribe(node, "/amiro2/potentialfield", 1);
      message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amiro_pf_sub1, amiro_pf_sub2);
      sync.registerCallback(boost::bind(&processMergeAllPotentialfieldsSynced, _1, _2));
      break;
    }
    case 3: {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
      amiro_pf_sub1.subscribe(node, "/amiro1/potentialfield", 1);
      amiro_pf_sub2.subscribe(node, "/amiro2/potentialfield", 1);
      amiro_pf_sub3.subscribe(node, "/amiro3/potentialfield", 1);
      message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amiro_pf_sub1, amiro_pf_sub2, amiro_pf_sub3);
      sync.registerCallback(boost::bind(&processMergeAllPotentialfieldsSynced, _1, _2, _3));
      break;
    }
    case 4: {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
      amiro_pf_sub1.subscribe(node, "/amiro1/potentialfield", 1);
      amiro_pf_sub2.subscribe(node, "/amiro2/potentialfield", 1);
      amiro_pf_sub3.subscribe(node, "/amiro3/potentialfield", 1);
      amiro_pf_sub4.subscribe(node, "/amiro4/potentialfield", 1);
      message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amiro_pf_sub1, amiro_pf_sub2, amiro_pf_sub3, amiro_pf_sub4);
      sync.registerCallback(boost::bind(&processMergeAllPotentialfieldsSynced, _1, _2, _3, _4));
      break;
    }
    case 5: {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
      amiro_pf_sub1.subscribe(node, "/amiro1/potentialfield", 1);
      amiro_pf_sub2.subscribe(node, "/amiro2/potentialfield", 1);
      amiro_pf_sub3.subscribe(node, "/amiro3/potentialfield", 1);
      amiro_pf_sub4.subscribe(node, "/amiro4/potentialfield", 1);
      amiro_pf_sub5.subscribe(node, "/amiro5/potentialfield", 1);
      message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amiro_pf_sub1, amiro_pf_sub2, amiro_pf_sub3, amiro_pf_sub4, amiro_pf_sub5);
      sync.registerCallback(boost::bind(&processMergeAllPotentialfieldsSynced, _1, _2, _3, _4, _5));
      break;
    }
    case 6: {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
      amiro_pf_sub1.subscribe(node, "/amiro1/potentialfield", 1);
      amiro_pf_sub2.subscribe(node, "/amiro2/potentialfield", 1);
      amiro_pf_sub3.subscribe(node, "/amiro3/potentialfield", 1);
      amiro_pf_sub4.subscribe(node, "/amiro4/potentialfield", 1);
      amiro_pf_sub5.subscribe(node, "/amiro5/potentialfield", 1);
      amiro_pf_sub6.subscribe(node, "/amiro6/potentialfield", 1);
      message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amiro_pf_sub1, amiro_pf_sub2, amiro_pf_sub3, amiro_pf_sub4, amiro_pf_sub5, amiro_pf_sub6);
      sync.registerCallback(boost::bind(&processMergeAllPotentialfieldsSynced, _1, _2, _3, _4, _5, _6));
      break;
    }
    case 7: {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
      amiro_pf_sub1.subscribe(node, "/amiro1/potentialfield", 1);
      amiro_pf_sub2.subscribe(node, "/amiro2/potentialfield", 1);
      amiro_pf_sub3.subscribe(node, "/amiro3/potentialfield", 1);
      amiro_pf_sub4.subscribe(node, "/amiro4/potentialfield", 1);
      amiro_pf_sub5.subscribe(node, "/amiro5/potentialfield", 1);
      amiro_pf_sub6.subscribe(node, "/amiro6/potentialfield", 1);
      amiro_pf_sub7.subscribe(node, "/amiro7/potentialfield", 1);
      message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amiro_pf_sub1, amiro_pf_sub2, amiro_pf_sub3, amiro_pf_sub4, amiro_pf_sub5, amiro_pf_sub6, amiro_pf_sub7);
      sync.registerCallback(boost::bind(&processMergeAllPotentialfieldsSynced, _1, _2, _3, _4, _5, _6, _7));
      break;
    }
    case 8: {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
      amiro_pf_sub1.subscribe(node, "/amiro1/potentialfield", 1);
      amiro_pf_sub2.subscribe(node, "/amiro2/potentialfield", 1);
      amiro_pf_sub3.subscribe(node, "/amiro3/potentialfield", 1);
      amiro_pf_sub4.subscribe(node, "/amiro4/potentialfield", 1);
      amiro_pf_sub5.subscribe(node, "/amiro5/potentialfield", 1);
      amiro_pf_sub6.subscribe(node, "/amiro6/potentialfield", 1);
      amiro_pf_sub7.subscribe(node, "/amiro7/potentialfield", 1);
      amiro_pf_sub8.subscribe(node, "/amiro8/potentialfield", 1);
      message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amiro_pf_sub1, amiro_pf_sub2, amiro_pf_sub3, amiro_pf_sub4, amiro_pf_sub5, amiro_pf_sub6, amiro_pf_sub7, amiro_pf_sub8);
      sync.registerCallback(boost::bind(&processMergeAllPotentialfieldsSynced, _1, _2, _3, _4, _5, _6, _7, _8));
      break;
    }
    case 9: {
      typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
      amiro_pf_sub1.subscribe(node, "/amiro1/potentialfield", 1);
      amiro_pf_sub2.subscribe(node, "/amiro2/potentialfield", 1);
      amiro_pf_sub3.subscribe(node, "/amiro3/potentialfield", 1);
      amiro_pf_sub4.subscribe(node, "/amiro4/potentialfield", 1);
      amiro_pf_sub5.subscribe(node, "/amiro5/potentialfield", 1);
      amiro_pf_sub6.subscribe(node, "/amiro6/potentialfield", 1);
      amiro_pf_sub7.subscribe(node, "/amiro7/potentialfield", 1);
      amiro_pf_sub8.subscribe(node, "/amiro8/potentialfield", 1);
      amiro_pf_sub9.subscribe(node, "/amiro9/potentialfield", 1);
      message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amiro_pf_sub1, amiro_pf_sub2, amiro_pf_sub3, amiro_pf_sub4, amiro_pf_sub5, amiro_pf_sub6, amiro_pf_sub7, amiro_pf_sub8, amiro_pf_sub9);
      sync.registerCallback(boost::bind(&processMergeAllPotentialfieldsSynced, _1, _2, _3, _4, _5, _6, _7, _8, _9));
      break;
    }
  }
  ros::spin();
  return 0;

}
