// ============================================================================
// Name        : rsb_to_ros_time_converter.cpp
// Author      : Daniel Rudolph <drudolph@techfak.uni-bielefeld.de>
// Description : Convert rsb timestamps into ros time.
// ============================================================================

#ifndef _rsb_to_ros_time_converter_
#define _rsb_to_ros_time_converter_

// RSB
#include <rsb/Event.h>
#include <rsb/MetaData.h>

// ROS
#include <ros/ros.h>

// BOOST
#include "boost/date_time/posix_time/posix_time.hpp"

  inline ros::Time rsbCreateTime2ros(const rsb::EventPtr event) {
    const rsb::MetaData md = event->getMetaData();
    const boost::uint64_t time_ms = md.getCreateTime();
    const boost::uint64_t msPerS = 1000000;
    const boost::uint64_t nsPerMs = 1000;
    return ros::Time(time_ms / msPerS, (time_ms % msPerS) * nsPerMs);
  }

  inline ros::Time getRosTimeFromRsbEvent(const rsb::EventPtr event = NULL, bool useRosTime = false) {
    if (useRosTime) {
        return ros::Time::now();
    } else if (event == NULL) {
        ROS_WARN("No RSB event given, using ROS time");
        return ros::Time::now();
    } else {
        return rsbCreateTime2ros(event);
    }
  }


#endif
