#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>


static const std::string WINDOW = "Test Window";

class LandingZoneDetection {
    private:
        // Node Handles
        ros::NodeHandle node;
        ros::NodeHandle _nh;

        // Parameters
        std::string depth_topic;

        // Image transport object used to convert depth image to OpenCV image
        image_transport::ImageTransport it;

        // Subscribers
        image_transport::Subscriber depth_subscriber;       // Subscriber used to subscribe to the depth camera info

    public:
        LandingZoneDetection() : it(node) {
            // Initialize the node handles
            _nh = ros::NodeHandle("~");

            // Load the system parameters
            _nh.getParam("depth_topic", depth_topic);

            // Initialize the depth subscriber and connect it to the correct callback function
            depth_subscriber = it.subscribe(depth_topic, 1, LandingZoneDetection::depth_callback, this);

            // Initialize a new OpenCV window
            cv::namedWindow(WINDOW, cv::WINDOW_AUTOSIZE);
        }

        ~LandingZoneDetection() {
            cv::destroyWindow(WINDOW);
        }        

        void depth_callback(const sensor_msgs::ImageConstPtr& image) {
            cv_bridge::CvImageConstPtr cv_ptr;

            try {
                cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_16UC1);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("[ERROR] Error encountered when copying the image to a CV Image (cv_bridge error): %s", e.what());
                return;
            }

            cv::imshow(WINDOW, cv_ptr->image);
            cv::waitKey(100);
        }
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "landing_zone_detector");
    LandingZoneDetection lz;
    ros::spin();
    return 0;
}