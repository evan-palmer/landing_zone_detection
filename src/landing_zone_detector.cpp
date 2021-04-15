#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>
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

            ROS_INFO("Distance at point (%d, %d): %f meters", x, y, distance);
        }

        
        /*
         * Get an average depth from the image to determine the distance from the frame
         */
        float get_average_altitude(const cv::Mat& image, float horizontal_range, float vertical_range) {
            float altitude = 0.0;     // Store the total distance for average computation
            float x, y;               // x -> random column selected, y -> random row selected
            float range = 10.0;       // The number of points to consider in the average calculation
            float actual_range = 0.0; // The actual number of points used (done to account for invalid pixels selected)
            float depth = 0.0;        // The depth measurement at the selected point
            
            for (int i = 0; i < range; ++i) {
                // Get the random points from the image to use for altitude calculation
                x = std::rand() % horizontal_range;
                y = std::rand() % vertical_range

                // Get the depth and convert it to centimeters
                float depth = 0.1 * image.at<u_int16_t>(y, x);

                // Discard the value if it is an invalid pixel
                if (depth == 0.0) {
                    continue;
                }

                // Update the mean calculation params
                altitude += depth;
                actual_range++;
            }

            return altitude / actual_range;
        }


        /*
         * Method responsible for computing maximum gradient in the image
         */
        float get_gradient(const cv::Mat& image, int columns, int rows) {
            float max_depth = 0.0;          // Max depth in the image (point furthest away)
            float min_depth = INFINITY;     // Min depth in the image (point closest)

            // Iterate through all pixels in the image
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < columns; ++j) {

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

            // Get the number of columns and rows in the image
            // The number of columns and rows in the image after conversion is used in case of discrepancy between message values
            // and converted image sizes
            int cols = (int)cv_ptr->image.cols;
            int rows = (int)cv_ptr->image.rows;

            // Get the average depth from the image to determine the approximate height
            float altitude = get_average_altitude(cv_ptr->image, cols - 1, rows - 1);

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
            horizontal_range -= (horizontal_range/cols) * idb;

            // Calculate the maximum depth gradient in the image
            float gradient = get_gradient(cv_ptr->image, cols, rows);

            ROS_INFO("Horizontal Range: %f  Vertical Range: %f  Max Gradient: %f", horizontal_range, vertical_range, gradient);

            // Draw idb
            cv::rectangle(cv_ptr->image, cv::Point2f(0, 0), cv::Point2f(idb - 1, rows - 1), 0xffff00, 2);

            // Display the depth image
            cv::imshow(WINDOW, cv_ptr->image);

            // Set the mouse callback to see distance measurements on click/hover
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
