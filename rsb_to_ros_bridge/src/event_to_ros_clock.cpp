// ROS
#include <ros/ros.h>
#include <sensor_msgs/TimeReference.h>
#include <rosgraph_msgs/Clock.h>

// RSB
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/MetaData.h>

// RST
#include <rst/vision/LaserScan.pb.h>
#include <rst/generic/Value.pb.h>
#include <rst/geometry/Pose.pb.h>

ros::Publisher publisher;
rosgraph_msgs::Clock clockMsg;
int useRsbSendTime;

// Callback for rsb events
// Reads timestamp from rsb event and republishes the timestamp to ros
void callback(rsb::EventPtr event) {
  rsb::MetaData data   = event->getMetaData();
  boost::uint64_t time = useRsbSendTime ? data.getSendTime() : data.getCreateTime();

  static boost::uint64_t lastTime;
  static boost::uint64_t newestTime;

  if (time < lastTime) {
    int diff = (int) (time - lastTime);
  } else  {
    if (time > newestTime) {
      clockMsg.clock = ros::Time((long double) time / 1e6);
      publisher.publish(clockMsg);
      newestTime = time;
    }
  }
  lastTime = time;
}

/**
 * This programm republishes timestamps from RSB messages as clock event in ros
 */
int main(int argc, char ** argv) {
  // ROS
  ros::init(argc, argv, "claas_bridge_clock");
  ros::NodeHandle n("~");

  std::string topicClock;
  std::string scope;

  n.param<std::string>("topic_clock", topicClock, "/clock");
  n.param<std::string>("scope_rsb", scope, "/some/scope");
  n.param<int>("use_rsb_send_time", useRsbSendTime, 0);

  publisher = n.advertise<rosgraph_msgs::Clock>(topicClock, 1);

  // RSB
  rsb::Factory& factory = rsb::getFactory();

  // TODO generalize to any rsb publisher
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::generic::Value> >
  converter1(new rsb::converter::ProtocolBufferConverter<rst::generic::Value>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter1);

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::geometry::Pose> >
  converter2(new rsb::converter::ProtocolBufferConverter<rst::geometry::Pose>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter2);

  // Prepare RSB listener
  rsb::ListenerPtr listener = factory.createListener(scope);
  listener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&callback)));

  ros::spin();

  return 0;
} // main
