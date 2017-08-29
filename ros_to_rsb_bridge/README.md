#ROS_TO_RSB_BRIDGE
This package contains the following converter from ros- to rsb-message types:
* ros_geometry_msgs_posestamped_to_rst_geometry_pose: ros::geometry_msgs::PoseStamped --> rst.geometry.pose
* ros_geometry_msgs_twist_to_rst_value_array: ros::geometry_msgs::Twist --> rst.generic.value
* ros_int_multiarray_rst_value_array: sai_msgs::Int32MultiArrayStamped --> rst.generic.value

### sai_msgs
This is a custom message type declaration for ros. It creates a Int32MultiArrayStamped which contains a normal ros::std_msgs::Header and a ros::std_msgs::Int32MultiArray.
