#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include "landing_zone_detection/LandingZone.h"
#include <stdlib.h>
#include <math.h>
#include <bits/stdc++.h>
#include <boost/circular_buffer.hpp>


static const std::string WINDOW = "Window";
static const std::string HELP = "HELP";

class LandingZoneDetector {
    private:
        // Node Handles
        ros::NodeHandle node;
        ros::NodeHandle _nh;

        // Arguments
        bool display;                   // Argument used to specify whether to display depth output
        bool debug;                     // Argument used to specify to print debug statements

        // Topics
        std::string depth_topic;        // Topic to read the depth images from
        std::string landing_zone_topic; // Topic to publish the gradient data to
        std::string imu_topic;          // Topic to read the IMU data from

        // Parameters
        float baseline;                 // Distance from the left imager to the right imager
        float horizontal_resolution;    // Horizontal resolution of the left and right imagers
        float horizontal_fov;           // Horizontal field-of-view of the left and right imagers
        float vertical_fov;             // Vertical field-of-view of the left and right imagers
        float diagonal_fov;             // Diagonal field-of-view of the left and right imagers
        float padding_multiplier;       // Parameter indicating the multiplier to apply to the width of the invalid depth band measurement (takes into account the adjacent invalid points)
        float gradient_threshold;       // Parameter representing the maximum allowable threshold prior to rejecting a landing zone candidate
        float horizontal_scale;         // Scale to restrict the horizontal fov to
        float vertical_scale;           // Scale to restrict the vertical fov to
        int cb_capacity;                // Capacity of the moving average circle buffer

        // Image transport object used to convert depth image to OpenCV image
        image_transport::ImageTransport it;

        // Subscriber used to subscribe to the depth camera info
        image_transport::Subscriber depth_subscriber;
        ros::Subscriber imu_subscriber;

        // Publisher used to publish the computed landing zone data
        ros::Publisher landing_zone_publisher;

        // Create a new cv bridge pointer
        cv_bridge::CvImageConstPtr cv_ptr;

        // Circle buffer to store gradients in for moving average calculation
        boost::circular_buffer<float> buffer;

        // Current Camera Orientation
        double roll, pitch, yaw;

        // Previous Camera Orientation (used for complementary filter)
        double prev_roll, prev_pitch, prev_yaw;

    public:
        LandingZoneDetector(bool debug_mode, bool display_mode) : it(node), debug(debug_mode), display(display_mode) {
            // Initialize the node handles
            _nh = ros::NodeHandle("~");

            // Load the system parameters
            _nh.getParam("depth_topic", depth_topic);
            _nh.getParam("landing_zone_topic", landing_zone_topic);
            _nh.getParam("imu_topic", imu_topic);
            _nh.getParam("baseline", baseline);
            _nh.getParam("hfov", horizontal_fov);
            _nh.getParam("vfov", vertical_fov);
            _nh.getParam("dfov", diagonal_fov);
            _nh.getParam("padding_multiplier", padding_multiplier);
            _nh.getParam("gradient_threshold", gradient_threshold);
            _nh.getParam("horiz_fov_scale", horizontal_scale);
            _nh.getParam("vert_fov_scale", vertical_scale);
            _nh.getParam("cb_capacity", cb_capacity);

            // Initialize the depth subscriber and connect it to the correct callback function
            depth_subscriber = it.subscribe(depth_topic, 1, &LandingZoneDetector::depth_callback, this);
            imu_subscriber = node.subscribe(imu_topic, 1, &LandingZoneDetector::imu_callback, this);

            // Initialize the landing zone publisher object to publish the computed landing zone data
            landing_zone_publisher = node.advertise<landing_zone_detection::LandingZone>(landing_zone_topic, 1);

            // Set the buffer capacity
            buffer.set_capacity(cb_capacity);

            // Initialize the elements to be 0.0
            for (int i = 0; i < buffer.capacity(); ++i) {
                buffer.push_back(0.0);
            }

            // Initialize Orientation
            roll, pitch, yaw = 0.0f;
            prev_roll, prev_pitch, prev_yaw = 0.0f;

            // Initialize a new OpenCV window
            cv::namedWindow(WINDOW);
        }

        /*
         * Destructor
         */
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
         * Static method called on mouse hover or click
         */
        static void onMouse(int event, int x, int y, int, void* detector) {
            // Cast the userparams (this) to a LandingZoneDetector object
            LandingZoneDetector* test = reinterpret_cast<LandingZoneDetector*>(detector);

            // Call the class method to display the desired data
            test->mouse_callback(event, x, y);
        }


        /*
         * Testing Purposes
         * Method used to get the distance at the selected point and display that to the user
         */
        void mouse_callback(int event, int x, int y) {
            // Get the distance at the selected point
            double distance = 0.1 * cv_ptr->image.at<u_int16_t>(y, x);

            ROS_INFO("Distance at point (%d, %d): %f centimeters", x, y, distance);
        }

        
        /*
         * Get an average depth from the image to determine the distance from the frame
         */
        float get_average_altitude(const cv::Mat& image) {           
            float altitude = 0.0;     // Store the total distance for average computation
            float actual_range = 0.0; // The actual number of points used (done to account for invalid pixels selected)
            
            // Get the average depth
            for (int i = 0; i < image.rows; ++i) {
                for (int j = 0; j < image.cols; ++j) {
                    // Get the depth and convert it to centimeters
                    float depth = 0.1 * image.at<u_int16_t>(i, j);

                    // Discard the value if it is an invalid pixel
                    if (depth == 0.0) {
                        continue;
                    }

                    // Update the mean calculation params
                    altitude += depth;
                    actual_range++;
                }                
            }

            return altitude / actual_range;
        }


        /*
         * Scale the image down to the desired size
         */
        cv::Mat get_scaled_image(const cv::Mat& image, int idb, float horizontal_scale, float vertical_scale) {
            // Calculate the number of pixels to extract
            float vertical_pixels = (float)image.rows * (1.0 - vertical_scale);
            float horizontal_pixels = (float)(image.cols - idb) * (1.0 - horizontal_scale);

            // Calculate the start and end pixels for the ROI
            int x = (int)floor(horizontal_pixels / 2);
            int y = (int)floor(vertical_pixels / 2);

            // Calculate a rectangle with the restricted field of view
            cv::Rect scale(x + idb, y, (image.cols - idb) - 2 * x, image.rows - 2 * y);
            
            // Apply the scale to the image
            cv::Mat scaled_image = image(scale);
            
            return scaled_image;
        }


        /*
         * Method responsible for computing maximum gradient in the image
         */
        float get_gradient(const cv::Mat& image) {
            float max_depth = 0.0;          // Max depth in the image (point furthest away)
            float min_depth = INFINITY;     // Min depth in the image (point closest)

            // Iterate through all pixels in the image
            for (int i = 0; i < image.rows; ++i) {
                for (int j = 0; j < image.cols; ++j) {

                    // Get the depth and convert it from mm to cm
                    float depth = 0.1 * image.at<u_int16_t>(i, j);

                    // Discard the value if it is an invalid depth measurement
                    if (depth == 0.0) {
                        continue;
                    }

                    // Update the min/max values accordingly
                    if (depth >= max_depth) {
                        max_depth = depth;
                    } else if (depth < min_depth) {
                        min_depth = depth;
                    }
                }
            }

            return max_depth - min_depth;
        }


        /*
         * Method responsible for computing the average of the gradient measurements in the circle buffer
         */
        float get_running_average_gradient(void) {
            float average_gradient = 0.0;

            for (int i = 0; i < buffer.capacity(); ++i) {
                average_gradient += buffer[i];
            }

            average_gradient /= buffer.capacity();

            return average_gradient;
        }


        /*
         * Callback function that handles processing the depth image
         */
        void depth_callback(const sensor_msgs::ImageConstPtr& msg) {
            // Convert the ROS Image msg into a cv pointer
            try {
                // This is a class parameter to enable the mouse callback
                // In the future, move the declaration to the method
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("[ERROR] Error encountered when copying the image to a CV Image (cv_bridge error): %s", e.what());
                return;
            }

            // Get the average depth from the image to determine the approximate height
            float altitude = get_average_altitude(cv_ptr->image);

            // Compute the invalid depth band ratio
            float dbr = compute_dbr(baseline, horizontal_fov, altitude);

            // Compute the pixels associated with the invalid depth band
            float idb = compute_idb(dbr, msg->width) * padding_multiplier;

            // Compute the distance-based diagonal fov
            float dfov = compute_diagonal_fov(horizontal_fov, baseline, altitude);

            // Compute the length of the diagonal
            float diagonal = compute_diagonal(altitude, dfov);

            // Calculate beta
            float beta = compute_beta((float)msg->width, (float)msg->height);

            // Apply the fov to distance mapping
            float horizontal_range = diagonal * sin(beta);
            float vertical_range = diagonal * cos(beta);

            // Subtract off the IDB from the field-of-view
            horizontal_range -= (horizontal_range/cv_ptr->image.cols) * idb;

            horizontal_range *= horizontal_scale;
            vertical_range *= vertical_scale;

            // Calculate the restricted image
            // NOTE: This also removes the IDB from the image
            cv::Mat restricted_image = get_scaled_image(cv_ptr->image, idb, horizontal_scale, vertical_scale);

            // Calculate the maximum depth gradient in the image
            float gradient = get_gradient(restricted_image);

            // Add the most recent gradient calculation to the circle buffer
            buffer.push_back(gradient);

            // Get the moving average
            float average_gradient = get_running_average_gradient();

            // Determine whether the landing zone is acceptable
            bool acceptable = average_gradient > gradient_threshold ? false : true;

            // Debug statement
            // ROS_INFO("Horizontal Range: %f  Vertical Range: %f  Max Gradient: %f  Average Gradient: %f", horizontal_range, vertical_range, gradient, average_gradient);

            // Create a new message to publish the computed gradient
            landing_zone_detection::LandingZone lz_msg;

            // // Fill out the landing zone message 
            lz_msg.header.stamp = ros::Time::now();
            lz_msg.header.frame_id = "/lz_camera";
            lz_msg.horizontal_fov = horizontal_range;
            lz_msg.vertical_fov = vertical_range;
            lz_msg.last_gradient = gradient;
            lz_msg.average_gradient = average_gradient;
            lz_msg.altitude = altitude;
            lz_msg.idb = idb;
            lz_msg.acceptable = acceptable;

            // Publish the landing zone data
            landing_zone_publisher.publish(lz_msg);

            // Draw idb
            cv::rectangle(cv_ptr->image, cv::Point2f(0, 0), cv::Point2f(idb - 1, cv_ptr->image.rows - 1), 0xffff00, 2);

            if (display) {
                // Display the depth image
                cv::imshow(WINDOW, cv_ptr->image);

                // Set the mouse callback to see distance measurements on click/hover
                cv::setMouseCallback(WINDOW, onMouse, this);
                cv::waitKey(100);
            }
        }

        void imu_callback(const sensor_msgs::ImuConstPtr& msg) {
            // pitch += (msg->angular_velocity.x )
        }
};




int main(int argc, char ** argv) {
    if (argc <= 1) {
        ROS_DEBUG("Invalid number of arguments provided at launch");
        return 0;
    }
    
    std::stringstream ss;
    
    bool debug_mode, display_mode;

    // Read the debug mode and set the respective variable value
    ss << argv[1];
    ss >> std::boolalpha >> debug_mode;

    // Read the display mode and set the respective variable value
    ss << argv[2];
    ss >> std::boolalpha >> display_mode;

    // Initialize the ROS  ode
    ros::init(argc, argv, "landing_zone_detector_node");
    
    // Instantiate a new LandingZoneDetector object to run
    LandingZoneDetector lz(debug_mode, display_mode);

    ros::spin();
    return 0;
}
