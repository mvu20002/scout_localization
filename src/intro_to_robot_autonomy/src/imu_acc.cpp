#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cmath>

// Publisher for accelerometer error
ros::Publisher accel_error_pub;

// Callback function for IMU data
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // Extract accelerometer data
    double accel_x = msg->linear_acceleration.x;
    double accel_y = msg->linear_acceleration.y;
    double accel_z = msg->linear_acceleration.z;

    // Theoretical stationary values (only gravity along Z-axis)
    double expected_accel_x = 0.0;
    double expected_accel_y = 0.0;
    double expected_accel_z = 9.81;  // Gravity acceleration in m/s^2

    // Calculate accelerometer error (Euclidean distance)
    double accel_error = std::sqrt(
        std::pow(accel_x - expected_accel_x, 2) +
        std::pow(accel_y - expected_accel_y, 2) +
        std::pow(accel_z - expected_accel_z, 2)
    );

    // Log the error
    ROS_INFO("Accelerometer Error: %.5f m/s^2", accel_error);

    // Publish the error
    std_msgs::Float64 error_msg;
    error_msg.data = accel_error;
    accel_error_pub.publish(error_msg);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "imu_accel_error_node");
    ros::NodeHandle nh;

    // Initialize the publisher
    accel_error_pub = nh.advertise<std_msgs::Float64>("/imu/accel_error", 10);

    // Subscribe to IMU data
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, imuCallback);

    // Spin to process callbacks
    ros::spin();

    return 0;
}
