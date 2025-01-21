#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv) {
  // ROS düğümünü başlat
  ros::init(argc, argv, "circle_mov_command");
  ros::NodeHandle nh;

  // 'cmd_vel' topiğine yayıncı oluştur
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  // Abonelik kontrolü
  ROS_INFO("Waiting for a subscriber to 'cmd_vel'...");
  while (vel_pub.getNumSubscribers() == 0) {
    ros::Duration(0.1).sleep();  // 100ms bekle
  }
  ROS_INFO("Subscriber detected. Starting circular motion...");

  // Yayınlama hızı (10 Hz)
  ros::Rate rate(10);

  // Twist mesajı oluştur
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = 0.6;  // İleri hız (m/s)
  vel_msg.angular.z = 0.05; // Açısal hız (rad/s)

  // Sürekli olarak hız komutlarını yayınla
  while (ros::ok()) {
    vel_pub.publish(vel_msg);

    // ROS döngüsü
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
