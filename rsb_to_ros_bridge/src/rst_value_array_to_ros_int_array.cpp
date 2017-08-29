// ============================================================================
// Name        : rst_value_array_to_ros_int_array.cpp
// Author      : Jonas Dominik Homburg <jhomburg@techfak.uni-bielefeld.de>
// Description : Recieve int32_t array as Value via rsb and publish them via ros.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <sai_msgs/Int32MultiArrayStamped.h>

// RSB
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/MetaData.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST
#include <rst/generic/Value.pb.h>

// #include "rsb_to_ros_time_converter.hpp"
#include <rsb_to_ros_bridge/rsb_to_ros_time_converter.h>

using namespace std;

// RSB Listener Scope
string rsbListenerScope;

// ROS Publish Topic
string rosPublishProximityTopic;

ros::Publisher floorProxPub;

bool rostimenow;

// program name
const string programName = "rst_value_array_to_ros_int_array";


void processValueArray(rsb::EventPtr event) {
  if (event->getType() != "rst::generic::Value") {
    return;
  }

  boost::shared_ptr<rst::generic::Value> value = boost::static_pointer_cast<rst::generic::Value>(event->getData());
  if (value->type() != rst::generic::Value::ARRAY) {
    return;
  }

  int size = value->array_size();
  std::vector<int32_t> data(0, 0);
  rst::generic::Value entry;
  for (int i = 0; i < size; i++) {
    entry = value->array(i);
    if (entry.type() != rst::generic::Value::INT) {
      return;
    }

    data.push_back(entry.int_());
    ROS_INFO("%i", data.back());
  }
  ROS_INFO("=======");

  std_msgs::MultiArrayLayout layout;
  std::vector<std_msgs::MultiArrayDimension> dimensions;
  std_msgs::MultiArrayDimension dim;
  dim.label  = "Proximity sensor values.";
  dim.size   = data.size();
  dim.stride = data.size();
  dimensions.push_back(dim);
  layout.dim = dimensions;


  sai_msgs::Int32MultiArrayStamped proxMsg;
  proxMsg.data.data       = data;
  proxMsg.data.layout     = layout;
  proxMsg.header.stamp    = getRosTimeFromRsbEvent(event,rostimenow);
  proxMsg.header.frame_id = event->getScope().getComponents()[0] + "/base_prox";

  floorProxPub.publish(proxMsg);
} // processValueArray

int main(int argc, char * argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  node.param<string>("rsb_listener_scope", rsbListenerScope, "/rir_prox/original");
  ROS_INFO("rsb_listener_scope: %s", rsbListenerScope.c_str());
  node.param<string>("ros_publish_topic", rosPublishProximityTopic, "/prox");
  ROS_INFO("ros_publish_topic: %s", rosPublishProximityTopic.c_str());
  node.param<bool>("rostimenow", rostimenow, false);
  ROS_INFO("rostimenow: %s", rostimenow?"True":"False");

  floorProxPub = node.advertise<sai_msgs::Int32MultiArrayStamped>(rosPublishProximityTopic, 1);

  rsb::Factory& factory = rsb::getFactory();

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::generic::Value> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::generic::Value>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB listener
  rsb::ListenerPtr floorProxListener = factory.createListener(rsbListenerScope);
  floorProxListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&processValueArray)));

  ros::spin();

  return 0;
}
