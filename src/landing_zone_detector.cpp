#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <math.h>

static const std::string WINDOW = "Window";

class LandingZoneDetector {
    private:
        // Node Handles
        ros::NodeHandle node;
        ros::NodeHandle _nh;

        // Parameters
        std::string depth_topic;        // Topic to read the depth images from
        float baseline;                 // Distance from the left imager to the right imager
        float horizontal_resolution;    // Horizontal resolution of the left and right imagers
        float horizontal_fov;           // Horizontal field-of-view of the left and right imagers
        float vertical_fov;             // Vertical field-of-view of the left and right imagers
        float diagonal_fov;             // Diagonal field-of-view of the left and right imagers
        float padding_multiplier;       // Parameter indicating the multiplier to apply to the width of the invalid depth band measurement (takes into account the adjacent invalid points)
        float test_distance;            // Parameter to apply for testing distance (In future will be altitude)

        // Image transport object used to convert depth image to OpenCV image
        image_transport::ImageTransport it;

        // Subscriber used to subscribe to the depth camera info
        image_transport::Subscriber depth_subscriber;

        // Create a new cv bridge pointer
        cv_bridge::CvImageConstPtr cv_ptr;

    public:
        LandingZoneDetector() : it(node) {
            // Initialize the node handles
            _nh = ros::NodeHandle("~");

            // Load the system parameters
            _nh.getParam("depth_topic", depth_topic);
            _nh.getParam("baseline", baseline);
            _nh.getParam("hfov", horizontal_fov);
            _nh.getParam("vfov", vertical_fov);
            _nh.getParam("dfov", diagonal_fov);
            _nh.getParam("padding_multiplier", padding_multiplier);
            _nh.getParam("test_distance", test_distance);

            // Initialize the depth subscriber and connect it to the correct callback function
            depth_subscriber = it.subscribe(depth_topic, 1, &LandingZoneDetector::depth_callback, this);

            // Initialize a new OpenCV window
            cv::namedWindow(WINDOW);
        }


        ~LandingZoneDetector() {
            cv::destroyWindow(WINDOW);
        }


        /*
         * Compute the depth band ratio using the horizontal field-of-view, distance, and baseline
         */
        float compute_dbr(float baseline, float hfov, float z) {
            return baseline / (2 * z * tan(hfov / 2));
        }


        /*
         * Compute the number of pixels that fall within the invalid depth band
         */
        float compute_idb(float dbr, float hres) {
            return ceil(dbr * hres);
        }


        /*
         * Compute the diagonal field-of-view for a given distance
         */
        float compute_diagonal_fov(float hfov, float baseline, float z) {
            return (hfov / 2) + atan(tan(hfov / 2) - (baseline / z));
        }


        /*
         * Compute the length of the diagonal according to a distance and diagonal field of view
         */
        float compute_diagonal(float z, float theta) {
            return 2 * z * tan(theta / 2);
        }


        /*
         * Compute beta (one of the angles in a right triangle produced by the horizontal, diagonal, and vertial widths)
         */ 
        float compute_beta(float width, float height) {
            return atan(width/height);
        }

        
        /*
         * Testing Purposes
         */
        static void onMouse(int event, int x, int y, int, void* detector) {
            LandingZoneDetector* test = reinterpret_cast<LandingZoneDetector*>(detector);
            test->mouse_callback(event, x, y);
        }


        /*
         * Testing Purposes
         */
        void mouse_callback(int event, int x, int y) {
            double distance = 0.001*cv_ptr->image.at<u_int16_t>(y, x);

            ROS_INFO("Distance at point (%d, %d): %f meters", x, y, distance);
        }

        
        float get_max_gradient(const cv::Mat& image) {
            // Matrices to store the gradients
            cv::Mat grad_x;
            cv::Mat grad_y;

            // Calculate the x gradient using the scharr function
            cv::Sobel(image, grad_x, -1, 1, 0, -1);

            // Calculate the y gradient using the scharr function
            cv::Sobel(image, grad_y, -1, 0, 1, -1);

            // Variables used to store min/max values
            double min_x_value, min_y_value;
            double max_x_value, max_y_value;

            // Variables used to store min/max locations
            cv::Point min_x_loc, min_y_loc;
            cv::Point max_x_loc, max_y_loc;

            cv::minMaxLoc(grad_x, &min_x_value, &max_x_value, &min_x_loc, &max_x_loc);
            cv::minMaxLoc(grad_y, &min_y_value, &max_y_value, &min_y_loc, &max_y_loc);

            float max_gradient = max_x_value > max_y_value ? max_x_value * 0.001 : max_y_value * 0.001;

            ROS_INFO("Max Gradient: %f", max_gradient);

            return max_gradient;
        }


        /*
         * Callback function that handles processing the depth image
         */
        void depth_callback(const sensor_msgs::ImageConstPtr& msg) {
            // Compute the invalid depth band ratio
            float dbr = compute_dbr(baseline, horizontal_fov, test_distance);

            // Compute the pixels associated with the invalid depth band
            float idb = compute_idb(dbr, msg->width) * padding_multiplier;

            // Compute the distance-based diagonal fov
            float dfov = compute_diagonal_fov(horizontal_fov, baseline, test_distance);

            // Compute the length of the diagonal
            float diagonal = compute_diagonal(test_distance, dfov);

            // Calculate beta
            float beta = compute_beta((float)msg->width, (float)msg->height);

            // Apply the fov to distance mapping
            float horizontal_range = diagonal * sin(beta);
            float vertical_range = diagonal * cos(beta);

            // Convert the ROS Image msg into a cv pointer
            try {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("[ERROR] Error encountered when copying the image to a CV Image (cv_bridge error): %s", e.what());
                return;
            }

            // Get the number of columns and rows in the image
            int cols = (int)cv_ptr->image.cols;
            int rows = (int)cv_ptr->image.rows;

            // double distance_top = 0.001*cv_ptr->image.at<u_int16_t>(0, cols/2);
            // double distance_bottom = 0.001*cv_ptr->image.at<u_int16_t>(rows - 1, cols/2);
            // double distance_left = 0.001*cv_ptr->image.at<u_int16_t>(rows/2, idb - 1);
            // double distance_right = 0.001*cv_ptr->image.at<u_int16_t>(rows/2, cols - 1);

            // ROS_INFO("Distance Top: %f  Distance Bottom: %f  Distance Left: %f  Distance Right: %f", distance_top, distance_bottom, distance_left, distance_right);

            // Draw idb
            cv::rectangle(cv_ptr->image, cv::Point2f(0, 0), cv::Point2f(idb - 1, rows - 1), 0xffff00, 2);

            // Left Box
            cv::rectangle(cv_ptr->image, cv::Point2f(idb, rows/2 - 5), cv::Point2f(5 + idb, rows/2 + 5), 0xffff, 3);

            // Right Box
            cv::rectangle(cv_ptr->image, cv::Point2f(cols - 5, rows/2 - 5), cv::Point2f(cols, rows/2 + 5), 0xffff, 3);

            // Top Box
            cv::rectangle(cv_ptr->image, cv::Point2f(cols/2 - 5, 0), cv::Point2f(cols/2 + 5, 5), 0xffff, 3);

            // Bottom Box
            cv::rectangle(cv_ptr->image, cv::Point2f(cols/2 - 5, rows - 5), cv::Point2f(cols/2 + 5, rows), 0xffff, 3);

            cv::imshow(WINDOW, cv_ptr->image);
            cv::setMouseCallback(WINDOW, onMouse, this);
            cv::waitKey(100);
        }
};




int main(int argc, char ** argv) {
    ros::init(argc, argv, "landing_zone_detector_node");
    LandingZoneDetector lz;
    ros::spin();
    return 0;
}
