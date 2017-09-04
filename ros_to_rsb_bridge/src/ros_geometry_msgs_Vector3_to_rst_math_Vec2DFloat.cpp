// ============================================================================
// Name        : ros_geometry_msgs_Vector3_to_rst_math_Vec2DFloat.cpp
// Author      : Timo Korthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : TBA
// ============================================================================

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

// RSB
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/MetaData.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST
#include <rst/math/Vec2DFloat.pb.h>

using namespace std;

// Ros Listener Topic
string rosListenerTopic;

// RSB Publish Scope
string rsbPublishScope;

rsb::Informer<rst::math::Vec2DFloat>::Ptr informer;

// program name
const string programName = "ros_geometry_msgs_Vector3_to_rst_math_Vec2DFloat";


void process(const geometry_msgs::Vector3::ConstPtr& msg) {
  rsb::Informer<rst::math::Vec2DFloat>::DataPtr rsbmsg(new rst::math::Vec2DFloat);

  // Copy the message
  rsbmsg->set_x(msg->x);
  rsbmsg->set_y(msg->y);
  
  // Publish
  informer->publish(rsbmsg);
}

int main(int argc, char * argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  node.param<string>("ros_listener_topic", rosListenerTopic, "/amiro/vector");
  node.param<string>("rsb_publish_scope", rsbPublishScope, "/motor/5");
  ROS_INFO("rsb_publish_scope: %s", rsbPublishScope.c_str());
  ROS_INFO("ros_listener_topic: %s", rosListenerTopic.c_str());

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::math::Vec2DFloat> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::math::Vec2DFloat>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  rsb::Factory& factory = rsb::getFactory();
  informer = factory.createInformer<rst::math::Vec2DFloat>(rsbPublishScope);

  ros::Subscriber sub = node.subscribe(rosListenerTopic, 1, process);

  ros::spin();

  return 0;
}
