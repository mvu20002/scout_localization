#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "robotics_hw1/MotorSpeed.h"

// Publisher'lar
ros::Publisher motor_speed_fl_pub;
ros::Publisher motor_speed_fr_pub;
ros::Publisher motor_speed_rl_pub;
ros::Publisher motor_speed_rr_pub;

// Callback fonksiyonu
void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->name.size() < 4 || msg->velocity.size() < 4) {
        ROS_WARN("JointState mesajında yeterli bilgi yok!");
        return;
    }

    // RPM'e çevirme faktörü: 1 rad/s = (60 / 2π) RPM
    const double rad_to_rpm = 60.0 / (2.0 * M_PI);

    // Teker hızlarını alın ve RPM'e çevirin
    double rpm_fl = msg->velocity[0] * rad_to_rpm; // Front Left
    double rpm_fr = msg->velocity[1] * rad_to_rpm; // Front Right
    double rpm_rl = msg->velocity[2] * rad_to_rpm; // Rear Left
    double rpm_rr = msg->velocity[3] * rad_to_rpm; // Rear Right

    // MotorSpeed mesajlarını oluşturun
    robotics_hw1::MotorSpeed motor_speed_fl, motor_speed_fr, motor_speed_rl, motor_speed_rr;

    ros::Time current_time = ros::Time::now();

    motor_speed_fl.header.stamp = current_time;
    motor_speed_fl.rpm = rpm_fl;

    motor_speed_fr.header.stamp = current_time;
    motor_speed_fr.rpm = rpm_fr;

    motor_speed_rl.header.stamp = current_time;
    motor_speed_rl.rpm = rpm_rl;

    motor_speed_rr.header.stamp = current_time;
    motor_speed_rr.rpm = rpm_rr;

    // Mesajları yayınlayın
    motor_speed_fl_pub.publish(motor_speed_fl);
    motor_speed_fr_pub.publish(motor_speed_fr);
    motor_speed_rl_pub.publish(motor_speed_rl);
    motor_speed_rr_pub.publish(motor_speed_rr);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_speed_publisher");
    ros::NodeHandle nh;

    // Publisher'ları oluşturun
    motor_speed_fl_pub = nh.advertise<robotics_hw1::MotorSpeed>("motor_speed_fl", 10);
    motor_speed_fr_pub = nh.advertise<robotics_hw1::MotorSpeed>("motor_speed_fr", 10);
    motor_speed_rl_pub = nh.advertise<robotics_hw1::MotorSpeed>("motor_speed_rl", 10);
    motor_speed_rr_pub = nh.advertise<robotics_hw1::MotorSpeed>("motor_speed_rr", 10);

    // joint_states için Subscriber
    ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 10, jointStatesCallback);

    ros::spin();
    return 0;
}
