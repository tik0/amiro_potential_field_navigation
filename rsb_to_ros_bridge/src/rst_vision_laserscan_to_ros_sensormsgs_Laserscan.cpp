// ============================================================================
// Name        : main.cxx
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Recieve images via rsb and publish them via ros.
// ============================================================================

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// RSB
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/MetaData.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST
#include <rst/vision/LaserScan.pb.h>

// #include "rsb_to_ros_time_converter.hpp"
#include <rsb_to_ros_bridge/rsb_to_ros_time_converter.h>

using namespace std;

// RSB Listener Scope
static string rsbListenerScope;

// ROS Publish Topic
static string rosPublishLaserScanTopic;

// ros::Publisher rosPublisher;
static ros::Publisher laserScanPublisher;

static double offset = 0.0;

bool rostimenow;

// program name
const string programName = "rst_vision_laserscan_to_ros_sensormsgs_Laserscan";

static const string rstLaserScan = "rst::vision::LaserScan";

void processLaserScan(rsb::EventPtr event) {
  // cout << event->getData() << endl;
  if (event->getType() == rstLaserScan) {
    boost::shared_ptr<rst::vision::LaserScan> rsbLaserScan = boost::static_pointer_cast<rst::vision::LaserScan>(event->getData());
    sensor_msgs::LaserScan rosLaserScan;
    // rosLaserScan.header.stamp.nsec = event->getMetaData().getCreateTime() * 1000;
    rosLaserScan.header.stamp    = getRosTimeFromRsbEvent(event,rostimenow);
    rosLaserScan.header.frame_id = event->getScope().getComponents()[0] + "/base_laser";

    rosLaserScan.angle_min       = offset;
    rosLaserScan.angle_max       = offset + rsbLaserScan->scan_angle();
    rosLaserScan.angle_increment = rsbLaserScan->scan_angle() / rsbLaserScan->scan_values().size();
    // rosLaserScan.time_increment
    // rosLaserScan.scan_time
    // rosLaserScan.intensities

    rosLaserScan.range_max = rsbLaserScan->scan_values(0);
    rosLaserScan.range_min = rsbLaserScan->scan_values(0);
    rosLaserScan.ranges.resize(rsbLaserScan->scan_values().size());
    for (int i = 0; i < rsbLaserScan->scan_values().size(); i++) {
      float value = rsbLaserScan->scan_values(i);
      rosLaserScan.ranges[i] = value;
      rosLaserScan.range_min = min(value, rosLaserScan.range_min);
      rosLaserScan.range_max = max(value, rosLaserScan.range_max);
    }
    // cout << "min value: " << rosLaserScan.range_min << " max value: " << rosLaserScan.range_max << "angle incre: " << rosLaserScan.angle_increment <<  endl;
    // publish ROS message
    laserScanPublisher.publish(rosLaserScan);
  }
} // processLaserScan

int main(int argc, char * argv[]) {
  ROS_INFO("Start: %s", programName.c_str());

  // Init ROS
  ros::init(argc, argv, programName);
  ros::NodeHandle node("~");

  node.param<string>("rsb_listener_scope", rsbListenerScope, "/laserscan");
  node.param<string>("ros_publish_topic", rosPublishLaserScanTopic, "/laserscan");
  node.param<double>("offset_scan", offset, 0.0);
  node.param<bool>("rostimenow", rostimenow, false);

  ROS_INFO("rsbListenerScope: %s", rsbListenerScope.c_str());
  ROS_INFO("ros_publish_laserscan_topic: %s", rosPublishLaserScanTopic.c_str());
  ROS_INFO("offset_scan: %f", offset);
  ROS_INFO("rostime now %d", rostimenow);
  ROS_INFO("rostimenow: %s", rostimenow?"True":"False");

  laserScanPublisher = node.advertise<sensor_msgs::LaserScan>(rosPublishLaserScanTopic, 1);

  // Create RSB factory
  rsb::Factory& factory = rsb::getFactory();

  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan> >
  converter(new rsb::converter::ProtocolBufferConverter<rst::vision::LaserScan>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB listener
  rsb::ListenerPtr laserScanListener = factory.createListener(rsbListenerScope);
  laserScanListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&processLaserScan)));

  ros::spin();

  return 0;
} // main
