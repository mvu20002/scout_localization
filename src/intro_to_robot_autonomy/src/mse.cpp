#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <intro_to_robot_autonomy/MSE.h>  // Özel mesajı içeren header
#include <tf/tf.h>
#include <vector>
#include <cmath>
#include <string>

class RealTimeMSECalculator {
public:
    RealTimeMSECalculator(const std::string& topic1, const std::string& topic2, const std::string& output_topic) {
        ros::NodeHandle nh;

        // Gelen topic isimlerine göre subscriber'ları oluştur
        ground_truth_sub = nh.subscribe(topic1, 10, &RealTimeMSECalculator::groundTruthCallback, this);
        ekf_output_sub = nh.subscribe(topic2, 10, &RealTimeMSECalculator::ekfOutputCallback, this);

        // Hesaplanan MSE'yi yayınlamak için publisher
        mse_pub = nh.advertise<intro_to_robot_autonomy::MSE>(output_topic, 10);

        ground_truth_received = false;
        ekf_output_received = false;

        ROS_INFO("Subscribed to topics: %s and %s", topic1.c_str(), topic2.c_str());
        ROS_INFO("Publishing MSE to topic: %s", output_topic.c_str());
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz döngü
        while (ros::ok()) {
            if (ground_truth_received && ekf_output_received) {
                publishMSE();
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Subscriber ground_truth_sub;
    ros::Subscriber ekf_output_sub;
    ros::Publisher mse_pub;

    std::vector<double> ground_truth; // [x, y]
    std::vector<double> ekf_output;   // [x, y]

    bool ground_truth_received;
    bool ekf_output_received;

    void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        ground_truth = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y
        };
        ground_truth_received = true;
    }

    void ekfOutputCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        ekf_output = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y
        };
        ekf_output_received = true;
    }

    void publishMSE() {
        intro_to_robot_autonomy::MSE mse_msg;

        // Header bilgisi
        mse_msg.header.stamp = ros::Time::now();
        mse_msg.header.frame_id = "base_link";

        // Anlık hatalar ve MSE hesaplama
        double mse = 0.0;
        if (ground_truth.size() == 2 && ekf_output.size() == 2) { // Sadece x ve y
            std::vector<double> instant_errors(2, 0.0);
            for (size_t i = 0; i < ground_truth.size(); ++i) {
                instant_errors[i] = ground_truth[i] - ekf_output[i];
                mse += instant_errors[i] * instant_errors[i];
            }
            mse /= ground_truth.size(); // Ortalama hata karesi

            mse_msg.mse = mse;
            mse_msg.instant_error_x = instant_errors[0];
            mse_msg.instant_error_y = instant_errors[1];
            mse_msg.total_xy = std::sqrt(
                std::pow(instant_errors[0], 2) +  // error_x^2
                std::pow(instant_errors[1], 2)   // error_y^2
            );

            // Mesajı yayınla
            mse_pub.publish(mse_msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "real_time_mse_node");

    ros::NodeHandle nh("~");

    // Parametreleri al
    std::string topic1, topic2, output_topic;
    nh.param<std::string>("topic1", topic1, "/ground_truth");
    nh.param<std::string>("topic2", topic2, "/ekf_output");
    nh.param<std::string>("output_topic", output_topic, "/mse_output");

    RealTimeMSECalculator mse_calculator(topic1, topic2, output_topic);
    mse_calculator.spin();
    return 0;
}
