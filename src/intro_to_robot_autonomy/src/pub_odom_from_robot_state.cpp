#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <algorithm>

// Global variables to store the robot's pose and name
double x = 0.0;
double y = 0.0;
double z = 0.0;
double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;

std::string robot_name = "scout/"; // Robot modelinin ismi burada tanımlanıyor
bool robot_found = false; // Robotun bulunup bulunmadığını kontrol etmek için

// Callback function for model_states subscriber
void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  // Find the index of the robot in the models array
  auto it = std::find(msg->name.begin(), msg->name.end(), robot_name);
  if (it != msg->name.end()) {
    size_t index = std::distance(msg->name.begin(), it);
    robot_found = true;

    // Get position
    x = msg->pose[index].position.x;
    y = msg->pose[index].position.y;
    z = msg->pose[index].position.z;

    // Get orientation (quaternion to Euler)
    tf::Quaternion q(
      msg->pose[index].orientation.x,
      msg->pose[index].orientation.y,
      msg->pose[index].orientation.z,
      msg->pose[index].orientation.w
    );
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
  } else {
    ROS_WARN_THROTTLE(1.0, "Robot model '%s' not found in /gazebo/model_states", robot_name.c_str());
    robot_found = false;
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "gazebo_odometry_publisher");
  ros::NodeHandle n;

  // Get the robot name parameter if available
  n.param<std::string>("robot_name", robot_name, "scout/");

  // Publisher for odometry
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  // Subscriber for model_states
  ros::Subscriber model_states_sub = n.subscribe("/gazebo/model_states", 10, modelStatesCallback);

  ros::Rate r(50.0);
  while (ros::ok()) {
    ros::spinOnce(); // Check for incoming messages

    if (robot_found) {
      ros::Time current_time = ros::Time::now();

      // Create quaternion from yaw
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

      // Publish the transform over tf
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = current_time;
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = x;
      odom_trans.transform.translation.y = y;
      odom_trans.transform.translation.z = z;
      odom_trans.transform.rotation = odom_quat;


      // Publish the odometry message over ROS
      nav_msgs::Odometry odom;
      odom.header.stamp = current_time;
      odom.header.frame_id = "odom";

      // Set the position
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = z;
      odom.pose.pose.orientation = odom_quat;

      // Set the velocity (if available, otherwise set to zero)
      odom.child_frame_id = "base_link";
      odom.twist.twist.linear.x = 0.0;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.linear.z = 0.0;
      odom.twist.twist.angular.x = 0.0;
      odom.twist.twist.angular.y = 0.0;
      odom.twist.twist.angular.z = 0.0;

      // Publish the message
      odom_pub.publish(odom);
    } else {
      ROS_WARN_THROTTLE(1.0, "Waiting for robot model '%s' to appear in /gazebo/model_states", robot_name.c_str());
    }

    r.sleep();
  }
}