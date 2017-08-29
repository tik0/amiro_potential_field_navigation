#RSB_TO_ROS_BRIDGE
This package contains the following converter from rsb- to ros-message types:
* rsb_twb_to_ros_navmsgs_odometry: TeleWorkBench messages --> ros::navmsgs::Odometry
* rst_pose_to_ros_navmsgs_odometry: rst.geometry.pose --> ros::navmsgs::Odometry
* rst_pose_to_ros_posestamped: rst.geometry.pose --> ros::geometry_msgs::PoseStamped
* rst_value_array_to_ros_int_array: rst.generic.value --> sai_msgs::Int32MultiArrayStamped.
* rst_vision_image_to_ros_sensormsgs_image: rst.vision.image --> ros::sensor_msgs::image
* rst_vision_laserscan_to_ros_sensormsgs_Laserscan: rst.vision.laserscan --> ros::sensor_msgs::laserscan

### sai_msgs
This is a custom message type declaration for ros. It creates a Int32MultiArrayStamped which contains a normal ros::std_msgs::Header and a ros::std_msgs::Int32MultiArray.
