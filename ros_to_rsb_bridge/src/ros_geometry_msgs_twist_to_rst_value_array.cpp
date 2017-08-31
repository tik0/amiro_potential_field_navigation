// ============================================================================
// Name        : ros_geometry_msgs_twist_to_rst_value_array.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve geometry_msgs::Twist for xbox-teleop-controll via ros and publish them via rsb.
//               Compatible with "motorControl". This publish boost::shared_ptr<rst::generic::Value > with 3 Values:
//               forward velocity (µm/ms), angular velocity (µrad/ms) and duration (µs)
// ============================================================================

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

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

using namespace std;

// Ros Listener Topic
string rosListenerTopic;

// RSB Publish Scope
string rsbPublishScope;

rsb::Informer<rst::generic::Value>::Ptr informer;

// program name
const string programName = "ros_geometry_msgs_twist_to_rst_value_array";

static int durationTime; // [µs]


void process(const geometry_msgs::Twist::ConstPtr& msg) {
  rsb::Informer<rst::generic::Value>::DataPtr rsbmsg(new rst::generic::Value);
  rsbmsg->set_type(rst::generic::Value::ARRAY);

  // forwardVelocity [µm/ms]
  rst::generic::Value * forwardVelocity = rsbmsg->add_array();
  forwardVelocity->set_type(rst::generic::Value::INT);
  forwardVelocity->set_int_(msg->linear.x * 1e6);

  // angularVelocity  [µrad/ms]
  rst::generic::Value * angularVelocity = rsbmsg->add_array();
  angularVelocity->set_type(rst::generic::Value::INT);
  angularVelocity->set_int_(msg->angular.z * 1e6);

  // duration [µs]
  rst::generic::Value * duration = rsbmsg->add_array();
  duration->set_type(rst::generic::Value::INT);
  duration->set_int_(durationTime);

  informer->publish(rsbmsg);
}

int main(int argc, char * argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  node.param<string>("ros_listener_topic", rosListenerTopic, "/teleop_velocity_smoother/raw_cmd_vel");
  node.param<string>("rsb_publish_scope", rsbPublishScope, "/motor/5");
  node.param<int>("duration", durationTime, 0); // [us]
  ROS_INFO("rsb_publish_scope: %s", rsbPublishScope.c_str());
  ROS_INFO("ros_listener_topic: %s", rosListenerTopic.c_str());
  ROS_INFO("duration: %d", durationTime);

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::generic::Value> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::generic::Value>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  rsb::Factory& factory = rsb::getFactory();
  informer = factory.createInformer<rst::generic::Value>(rsbPublishScope);

  ros::Subscriber sub = node.subscribe(rosListenerTopic, 1, process);

  ros::spin();

  return 0;
}
