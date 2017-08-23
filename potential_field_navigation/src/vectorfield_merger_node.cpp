// ============================================================================
// Name        : vectorfield_merger_node.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
//               Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Receive all vectorfields and merger them.
// ============================================================================

#include <mutex>

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
const std::size_t numFields = 3;
static std::vector<cv::Mat> fields(numFields);
static std::vector<bool> dataArrived(numFields,false);
std::mutex dataArrivedMtx;

void processImage(const sensor_msgs::ImageConstPtr& msg) {
  const std::size_t id = 0;
  dataArrivedMtx.lock();
  dataArrived.at(id) = true;
  cv_bridge::toCvShare(msg, msg->encoding)->image.copyTo(fields.at(id));
  ROS_DEBUG_STREAM("processImage, size (x,y): " << fields.at(id).cols << " "  << fields.at(id).rows);
  dataArrivedMtx.unlock();
}

void processAmiro1(const sensor_msgs::ImageConstPtr& msg) {
  const std::size_t id = 1;
  dataArrivedMtx.lock();
  dataArrived.at(id) = true;
  cv_bridge::toCvShare(msg, msg->encoding)->image.copyTo(fields.at(id));
  ROS_DEBUG_STREAM("processAmiro1, size (x,y): " << fields.at(id).cols << " "  << fields.at(id).rows);
  dataArrivedMtx.unlock();
}

void processAmiro2(const sensor_msgs::ImageConstPtr& msg) {
  const std::size_t id = 2;
  dataArrivedMtx.lock();
  dataArrived.at(id) = true;
  cv_bridge::toCvShare(msg, msg->encoding)->image.copyTo(fields.at(id));
  ROS_DEBUG_STREAM("processAmiro2, size (x,y): " << fields.at(id).cols << " "  << fields.at(id).rows);
  dataArrivedMtx.unlock();
}

int main(int argc, char *argv[]) {
  ROS_INFO("Start: %s", ros::this_node::getName().c_str());
  // Init ROS
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle node("~");

  string vectorfieldPublisherTopic;
  string imageVectorfieldListenerTopic, amiro1VectorfieldListenerTopic, amiro2VectorfieldListenerTopic;
  int fieldWidth, fieldHeight;
  node.param<string>("image_vectorfield_listener_topic", imageVectorfieldListenerTopic, "/vectorfield/image");
  node.param<string>("image_amiro1_listener_topic", amiro1VectorfieldListenerTopic, "/vectorfield/amiro1");
  node.param<string>("image_amiro2_listener_topic", amiro2VectorfieldListenerTopic, "/vectorfield/amiro2");
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

  image_transport::ImageTransport imageTransport(node);
  image_transport::ImageTransport amiro1Transport(node);
  image_transport::ImageTransport amiro2Transport(node);
  imagePublisher = imageTransport.advertise(vectorfieldPublisherTopic, 1, true);
  image_transport::Subscriber image_sub = imageTransport.subscribe(imageVectorfieldListenerTopic, 1, &processImage);
  image_transport::Subscriber amiro1_sub = amiro1Transport.subscribe(amiro1VectorfieldListenerTopic, 1, &processAmiro1);
  image_transport::Subscriber amiro2_sub = amiro2Transport.subscribe(amiro2VectorfieldListenerTopic, 1, &processAmiro2);

  bool burnIn = true;
  ros::Rate rate(1);
  while(ros::ok()) {
    dataArrivedMtx.lock();
    if (!burnIn) {
      bool doFusion = false;
      for (auto it = dataArrived.begin(); it < dataArrived.end(); ++it) {
        if (*it) {
          doFusion = true;
          *it = false;
        }
      }
      if (doFusion) {
        ROS_INFO_STREAM(ros::this_node::getName() << "STATE: Do fusion");
        // Reset the old map
        vectorfield_merged.setTo(cv::Scalar(cv::Vec2f(0.0f, 0.0f)));
        // Fuse
        for (auto it = fields.begin(); it < fields.end(); ++it) {
          if (it->cols != fieldWidth || it->rows != fieldHeight) {
            ROS_WARN_STREAM(ros::this_node::getName() << " insufficient field size");
            continue;
          }
          vectorfield_merged = vectorfield_merged + *it;
        }
        // Publish
        cvImage.image = vectorfield_merged;
        imagePublisher.publish(cvImage.toImageMsg());
      } else {
        ROS_DEBUG_STREAM(ros::this_node::getName() << " STATE: Skip fusion");
      }
    } else {
      ROS_INFO_STREAM(ros::this_node::getName() << " STATE: Burn in");
      burnIn = false;
      for (auto it = dataArrived.begin(); it < dataArrived.end(); ++it) {
        if (*it == false) {
          burnIn = true; // Reset if any data haven't arrived yet
        }
      }
    }
    dataArrivedMtx.unlock();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
