#include <ros/ros.h>
#include <sensor_msgs/Image.h>


class LandingZoneDetection {
    private:
        // Node Handles
        ros::NodeHandle node;
        ros::NodeHandle _nh;

        // Subscribers
        ros::Subscriber depth_subscriber;       // Subscriber used to subscribe to the depth camera info

        // Parameters
        std::string depth_topic;

    public:
        LandingZoneDetection() {
            // Initialize the node handles
            node = ros::NodeHandle();
            _nh = ros::NodeHandle("~");

            // Load the system parameters
            _nh.getParam("depth_topic", depth_topic);

            // Initialize the depth subscriber and connect it to the correct callback function
            depth_subscriber = node.subscribe(depth_topic, 1, LandingZoneDetection::depth_callback, this);
        }

        void depth_callback(const sensor_msgs::ImageConstPtr& image) {
            ROS_INFO("Receiving Data");
        }
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "landing_zone_detector");
    LandingZoneDetection lz;
    ros::spin();
    return 0;
}