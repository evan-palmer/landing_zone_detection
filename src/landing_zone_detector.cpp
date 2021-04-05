#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <math.h>

static const std::string WINDOW = "Window";

class LandingZoneDetection {
    private:
        // Node Handles
        ros::NodeHandle node;
        ros::NodeHandle _nh;

        // Parameters
        std::string depth_topic;
        float baseline;
        float horizontal_resolution;
        float horizontal_fov;
        float vertical_fov;
        float diagonal_fov;

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
            _nh.getParam("baseline", baseline);
            _nh.getParam("hfov", horizontal_fov);
            _nh.getParam("vfov", vertical_fov);
            _nh.getParam("dfov", diagonal_fov);

            // Initialize the depth subscriber and connect it to the correct callback function
            depth_subscriber = it.subscribe(depth_topic, 1, &LandingZoneDetection::depth_callback, this);

            // Initialize a new OpenCV window
            cv::namedWindow(WINDOW);
        }

        // Destructor
        ~LandingZoneDetection() {
            cv::destroyWindow(WINDOW);
        }


        // Compute the Depth-band-ratio
        float compute_dbr(float baseline, float hfov, float z) {
            return baseline / (2 * z * tan(hfov / 2));
        }


        // Compute the distance-based diagonal fov
        float compute_diagonal_fov(float hfov, float baseline, float z) {
            return (hfov / 2) + atan(tan(hfov / 2) - (baseline / z));
        }


        // Compute the invalid depth band
        float compute_idb(float dbr, float hres) {
            return ceil(dbr * hres);
        }


        // Compute the length of the diagonal according to a distance and diagonal field of view
        float compute_diagonal(float z, float theta) {
            return 2 * z * tan(theta / 2);
        }


        void depth_callback(const sensor_msgs::ImageConstPtr& msg) {
<<<<<<< HEAD
            float test_distance = 2000;

            // Compute the invalid depth band ratio
            float dbr = compute_dbr(baseline, horizontal_fov, test_distance);

            // Compute the pixels associated with the invalid depth band
            float idb = compute_idb(dbr, msg->width);

            // Compute the distance-based diagonal fov
            float dfov = compute_diagonal_fov(horizontal_fov, baseline, test_distance);

            // Compute the length of the diagonal
            float diagonal = compute_diagonal(test_distance, dfov);

            // Apply the fov to distance mapping
            float horizontal_range = diagonal * sin(atan((msg->width)/(msg->height)));
            float vertical_range = diagonal * cos(atan((msg->width)/(msg->height)));

=======
            cv_bridge::CvImageConstPtr cv_ptr;
>>>>>>> 1a2578bd4575cfba1e2f6850550d9d6630058795

            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            } catch (cv_bridge::Exception& e) {
                    ROS_ERROR("[ERROR] Error encountered when copying the image to a CV Image (cv_bridge error): %s", e.what());
                    return;
            }

            int cols = (int)cv_ptr->image.cols;
            int rows = (int)cv_ptr->image.rows;

            //double distance_top = 0.001*cv_ptr->image.at<u_int16_t>(0, cols/2);
            double distance_bottom = 0.001*cv_ptr->image.at<u_int16_t>(rows, cols/2);
            //double distance_left = 0.001*cv_ptr->image.at<u_int16_t>(rows/2, 0);
            //double distance_right = 0.001*cv_ptr->image.at<u_int16_t>(rows/2, cols);

            ROS_INFO("Distance Bottom: %f", distance_bottom);
            //cv::putText(cv_ptr->image, std::to_string(distance), cv::Point(10, cv_ptr->image.rows/2), cv::FONT_HERSHEY_DUPLEX, 0.6, 0xffff, 2);

            // Left Box
            cv::rectangle(cv_ptr->image, cv::Point2f(0, rows/2 - 5), cv::Point2f(5, rows/2 + 5), 0xffff, 3);

            // Right Box
            cv::rectangle(cv_ptr->image, cv::Point2f(cols - 5, rows/2 - 5), cv::Point2f(cols, rows/2 + 5), 0xffff, 3);

            // Top Box
            cv::rectangle(cv_ptr->image, cv::Point2f(cols/2 - 5, 0), cv::Point2f(cols/2 + 5, 5), 0xffff, 3);

            // Bottom Box
            cv::rectangle(cv_ptr->image, cv::Point2f(cols/2 - 5, rows - 5), cv::Point2f(cols/2 + 5, rows), 0xffff, 3);

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
