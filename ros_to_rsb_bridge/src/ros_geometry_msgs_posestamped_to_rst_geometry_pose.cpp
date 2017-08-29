// ============================================================================
// Name        : ros_geometry_msgs_posestamped_to_rst_geometry_pose.cpp
// Author      : Jonas Dominik Homburg <jhomburg@techfak.uni-bielefeld.de>
// Description : Recieve a ros PoseStamped and publish it as rst geometry pose.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


// RSB
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/MetaData.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST
#include <rst/geometry/Pose.pb.h>

using namespace std;

// Ros Listener Topic
string rosListenerTopic;

// RSB Publish Scope
string rsbPublishScope;

rsb::Informer<rst::geometry::Pose>::Ptr informer;

// program name
const string programName = "ros_geometry_msgs_posestamped_to_rst_geometry_pose";


void process(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  rsb::Informer<rst::geometry::Pose>::DataPtr rstPose(new rst::geometry::Pose);

  rstPose->mutable_translation()->set_x(msg->pose.position.x);
  rstPose->mutable_translation()->set_y(msg->pose.position.y);
  rstPose->mutable_translation()->set_z(msg->pose.position.z);

  rstPose->mutable_rotation()->set_qx(msg->pose.orientation.x);
  rstPose->mutable_rotation()->set_qy(msg->pose.orientation.y);
  rstPose->mutable_rotation()->set_qz(msg->pose.orientation.z);
  rstPose->mutable_rotation()->set_qw(msg->pose.orientation.w);

  informer->publish(rstPose);
}

int main(int argc, char * argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  node.param<string>("ros_listener_topic", rosListenerTopic, "/pose");
  node.param<string>("rsb_publish_scope", rsbPublishScope, "/pose");
  ROS_INFO("ros_listener_topic: %s", rosListenerTopic.c_str());
  ROS_INFO("rsb_publish_topic: %s", rsbPublishScope.c_str());

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  rsb::Factory& factory = rsb::getFactory();
  informer = factory.createInformer<rst::geometry::Pose>(rsbPublishScope);

  ros::Subscriber sub = node.subscribe(rosListenerTopic, 1, process);

  ros::spin();

  return 0;
}
