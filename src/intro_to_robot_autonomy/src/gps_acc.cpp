#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <cmath>

// Function to calculate the distance between two GPS points using the Haversine formula
double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371e3; // Earth's radius in meters
    double phi1 = lat1 * M_PI / 180.0; // Convert latitude to radians
    double phi2 = lat2 * M_PI / 180.0;
    double delta_phi = (lat2 - lat1) * M_PI / 180.0;
    double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(delta_phi / 2.0) * sin(delta_phi / 2.0) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda / 2.0) * sin(delta_lambda / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return R * c; // Distance in meters
}

class GpsAccuracyChecker {
private:
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    ros::Publisher accuracy_pub;

    double ref_lat, ref_lon; // Reference GPS coordinates

public:
    GpsAccuracyChecker() {
        gps_sub = nh.subscribe("/gps/fix", 10, &GpsAccuracyChecker::gpsCallback, this);
        accuracy_pub = nh.advertise<std_msgs::Float64>("/gps/position/accuracy", 10);
        // Set the reference point (latitude and longitude)
        ref_lat = 41.105078;  // Example reference latitude
        ref_lon = 29.023579; // Example reference longitude
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
        if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
            ROS_WARN("GPS fix not available.");
            return;
        }

        // Calculate the distance to the reference point
        double distance = haversineDistance(ref_lat, ref_lon, msg->latitude, msg->longitude);

        // Publish the distance to the /gps/position/accuracy topic
        std_msgs::Float64 distance_msg;
        distance_msg.data = distance;
        accuracy_pub.publish(distance_msg);

        ROS_INFO("Real-time GPS Accuracy (meters): %f", distance);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_accuracy_checker");
    GpsAccuracyChecker checker;
    ros::spin();
    return 0;
}
