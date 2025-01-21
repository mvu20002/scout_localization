#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <exception>

// Yapı: Her hız komutunu ve süresini temsil eder
struct VelocityCommand {
    double linear_x;
    double linear_y;
    double angular_z;
    double duration;
};

int main(int argc, char** argv) {
    // ROS Node'u Başlat
    ros::init(argc, argv, "parametric_velocity_sequential");
    ros::NodeHandle nh;

    // cmd_vel için Publisher
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // YAML Dosyasını Alma
    std::string yaml_file;
    nh.param<std::string>("yaml_file", yaml_file, "/home/veli/scout_ws/src/intro_to_robot_autonomy/config/vel_commands.yaml");

    // YAML Dosyasını Yükle
    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_file);
    } catch (const YAML::BadFile& e) {
        ROS_ERROR("Failed to load YAML file: %s", yaml_file.c_str());
        return 1;
    } catch (const YAML::ParserException& e) {
        ROS_ERROR("YAML parsing error: %s", e.what());
        return 1;
    }

    // YAML Dosyasında 'commands' Alanını Kontrol Et
    if (!config["commands"]) {
        ROS_ERROR("YAML file does not contain 'commands' entry.");
        return 1;
    }

    // Hız Komutlarını Yükleme
    std::vector<VelocityCommand> velocity_commands;
    try {
        for (const auto& command : config["commands"]) {
            VelocityCommand vel_cmd;
            vel_cmd.linear_x = command["linear_x"].as<double>();
            vel_cmd.linear_y = command["linear_y"].as<double>();
            vel_cmd.angular_z = command["angular_z"].as<double>();
            vel_cmd.duration = command["duration"].as<double>();
            if (vel_cmd.duration <= 0) {
                ROS_WARN("Skipping command with non-positive duration: %f seconds.", vel_cmd.duration);
                continue;
            }
            velocity_commands.push_back(vel_cmd);
        }
    } catch (const YAML::Exception& e) {
        ROS_ERROR("Error parsing velocity commands from YAML: %s", e.what());
        return 1;
    }

    if (velocity_commands.empty()) {
        ROS_WARN("No valid velocity commands loaded from YAML file.");
        return 0;
    }

    ROS_INFO("Loaded %lu velocity commands.", velocity_commands.size());

    // Her Komutu Sırayla Yayınla
    ros::Rate rate(10); // 10 Hz
    for (const auto& vel_cmd : velocity_commands) {
        ROS_INFO("Publishing command: linear_x=%f, linear_y=%f, angular_z=%f for %f seconds.",
                 vel_cmd.linear_x, vel_cmd.linear_y, vel_cmd.angular_z, vel_cmd.duration);

        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = vel_cmd.linear_x;
        cmd_vel_msg.linear.y = vel_cmd.linear_y;
        cmd_vel_msg.angular.z = vel_cmd.angular_z;

        ros::Time start_time = ros::Time::now();
        ros::Duration duration_time(vel_cmd.duration);

        // Belirli Süre İçin Komutu Yayınla
        while (ros::ok() && (ros::Time::now() - start_time) < duration_time) {
            cmd_vel_pub.publish(cmd_vel_msg);
            rate.sleep();
        }
    }

    // Liste Bittikten Sonra Robotu Durdur
    ROS_INFO("All commands completed. Stopping the robot...");
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.linear.y = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub.publish(stop_msg);

    return 0;
}
