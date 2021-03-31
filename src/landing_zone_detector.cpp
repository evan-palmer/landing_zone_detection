#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>


static const std::string WINDOW = "Cool Window";

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
            depth_subscriber = it.subscribe(depth_topic, 1, &LandingZoneDetection::depth_callback, this);

            // Initialize a new OpenCV window
            cv::namedWindow(WINDOW, cv::WINDOW_AUTOSIZE);
        }

        ~LandingZoneDetection() {
            cv::destroyWindow(WINDOW);
        }        

        void depth_callback(const sensor_msgs::ImageConstPtr& image) {
            cv_bridge::CvImageConstPtr cv_ptr;
	    double distance_top = 0;	
	    double distance_bottom = 0;
	    double distance_left = 0;
	    double distance_right = 0;

            try {
                cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_16UC1);
            	distance_top = 0.001*cv_ptr->image.at<u_int16_t>(0, cv_ptr->image.cols/2);
                distance_bottom = 0.001*cv_ptr->image.at<u_int16_t>(cv_ptr->image.rows, cv_ptr->image.cols/2);
                distance_left = 0.001*cv_ptr->image.at<u_int16_t>(cv_ptr->image.rows/2, 0);
                distance_right = 0.001*cv_ptr->image.at<u_int16_t>(cv_ptr->image.rows/2, cv_ptr->image.cols);
                ROS_INFO("Distance Top: %f  Distance Bottom: %f  Distance Left: %f  Distance Right: %f", distance_top, distance_bottom, distance_left, distance_right);
                //cv::putText(cv_ptr->image, std::to_string(distance), cv::Point(10, cv_ptr->image.rows/2), cv::FONT_HERSHEY_DUPLEX, 0.6, 0xffff, 2);
                
                // Top center box
                cv::rectangle(cv_ptr->image, cv::Point2f(0, <u_int_16_t>(cv_ptr->image.cols/2) - 5), cv::Point2f(5, <u_int_16_t>(cv_ptr->image.cols/2) + 5), 0xffff, 3);

                // Bottom center box
                cv::rectangle(cv_ptr->image, cv::Point2f(<u_int_16_t>(cv_ptr->image.rows) - 5, <u_int_16_t>(cv_ptr->image.cols/2) - 5), cv::Point2f(<u_int_16_t>(cv_ptr->image.rows), <u_int_16_t>(cv_ptr->image.cols/2) + 5), 0xffff, 3);

                // Left box
                cv::rectangle(cv_ptr->image, cv::Point2f(<u_int_16_t>(cv_ptr->image.rows/2) - 5, 0), cv::Point2f(<u_int_16_t>(cv_ptr->image/2) + 5, 5), 0xffff, 3);
                
                // Right box
                cv::rectangle(cv_ptr->image, cv::Point2f(<u_int_16_t>(cv_ptr->image.rows/2) - 5, <u_int_16_t>(cv_ptr->image.cols) - 5), cv::Point2f(<u_int_16_t>(cv_ptr->image.rows/2) + 5, <u_int_16_t>(cv_ptr->image.cols)), 0xffff, 3);


            } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("[ERROR] Error encountered when copying the image to a CV Image (cv_bridge error): %s", e.what());
                    return;
            }

           cv::imshow(WINDOW, cv_ptr->image);
           cv::waitKey(100);
        }
};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "landing_zone_detector_node");
    LandingZoneDetection lz;
    ros::spin();
    return 0;
}
