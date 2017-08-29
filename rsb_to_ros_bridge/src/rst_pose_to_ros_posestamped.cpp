// ============================================================================
// Name        : rst_pose_to_ros_posestamped.cpp
// Author      : Jonas Dominik Homburg <jhomburg@techfak.uni-bielefeld.de>
// Description : Recieve int32_t array as Value via rsb and publish them via ros.
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

// #include "rsb_to_ros_time_converter.hpp"
#include <rsb_to_ros_bridge/rsb_to_ros_time_converter.h>

using namespace std;

// RSB Listener Scope
string rsbListenerScope;

// ROS Publish Topic
string rosPublishPoseStamped;

ros::Publisher rosPosePub;

// program name
const string programName = "rst_pose_to_ros_posestamped";

bool rostimenow;

void processRstMessage(rsb::EventPtr event) {
  if (event->getType() != "rst::geometry::Pose") {
    return;
  }

  boost::shared_ptr<rst::geometry::Pose> value = boost::static_pointer_cast<rst::geometry::Pose>(event->getData());
  rst::geometry::Translation t = value->translation();
  rst::geometry::Rotation r    = value->rotation();

  geometry_msgs::PoseStamped pS;
  pS.pose.orientation.x = (double) r.qx();
  pS.pose.orientation.y = (double) r.qy();
  pS.pose.orientation.z = (double) r.qz();
  pS.pose.orientation.w = (double) r.qw();
  pS.pose.position.x    = (double) t.x();
  pS.pose.position.y    = (double) t.y();
  pS.pose.position.z    = (double) t.z();
  pS.header.stamp       = getRosTimeFromRsbEvent(event,rostimenow);
  pS.header.frame_id    = event->getScope().getComponents()[0] + "/odom";

  rosPosePub.publish(pS);

  /*if (value->type() != rst::generic::Value::ARRAY){
   * return;
   * }
   *
   * int size = value->array_size();
   * std::vector<int32_t> data (0, 0);
   * rst::generic::Value entry;
   * for (int i=0; i<size; i++){
   * entry=value->array(i);
   * if (entry.type()!=rst::generic::Value::INT){
   *  return;
   * }
   *
   * data.push_back(entry.int_());
   * ROS_INFO("%i", data.back());
   * }
   * ROS_INFO("=======");
   *
   * std_msgs::MultiArrayLayout layout;
   * std::vector<std_msgs::MultiArrayDimension> dimensions;
   * std_msgs::MultiArrayDimension dim;
   * dim.label="Proximity sensor values.";
   * dim.size=data.size();
   * dim.stride=data.size();
   * dimensions.push_back(dim);
   * layout.dim=dimensions;
   *
   *
   * sai_msgs::Int32MultiArrayStamped proxMsg;
   * proxMsg.data.data=data;
   * proxMsg.data.layout=layout;
   * proxMsg.header.stamp.nsec=event->getMetaData().getCreateTime()*1000;
   * proxMsg.header.frame_id=event->getScope().toString();
   *
   * floorProxPub.publish(proxMsg);*/
}

int main(int argc, char * argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  node.param<string>("rsb_listener_scope", rsbListenerScope, "/prox");
  ROS_INFO("rsb_listener_scope: %s", rsbListenerScope.c_str());
  node.param<string>("ros_publish_topic", rosPublishPoseStamped, "/pose");
  ROS_INFO("ros_publish_topic: %s", rosPublishPoseStamped.c_str());
  node.param<bool>("rostimenow", rostimenow, false);
  ROS_INFO("rostimenow: %s", rostimenow?"True":"False");

  rosPosePub = node.advertise<geometry_msgs::PoseStamped>(rosPublishPoseStamped, 1);

  rsb::Factory& factory = rsb::getFactory();

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB listener
  rsb::ListenerPtr poseListener = factory.createListener(rsbListenerScope);
  poseListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&processRstMessage)));

  ros::spin();

  return 0;
}
