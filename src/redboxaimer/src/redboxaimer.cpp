#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <cmath>

class RedBoxAimer : public rclcpp::Node {
public:
    RedBoxAimer() : Node("red_box_aimer") {
        // Create subscriptions and publishers
        imagesub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/robotcam", 9, std::bind(&RedBoxAimer::image_callback, this, std::placeholders::_1));
        anglesub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/current_angle", 9, std::bind(&RedBoxAimer::angle_callback, this, std::placeholders::_1));
        anglepub_ = this->create_publisher<std_msgs::msg::Float32>("/desired_angle", 10);

        // Constants
        camera_fov_rad_ = M_PI / 2; // 90° FOV in radians
        max_yaw_correction_ = camera_fov_rad_ / 2; // ±π/2 max correction
        current_angle_ = 0.0; // Latest received yaw angle
    }

private:
    void angle_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        current_angle_ = msg->data;
        // RCLCPP_INFO(this->get_logger(), "curr angle: %f", current_angle_);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Convert ROS Image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat cv_image = cv_ptr->image;

        // Convert to HSV
        cv::Mat hsv;
        cv::cvtColor(cv_image, hsv, cv::COLOR_BGR2HSV);

        // Red mask
        cv::Scalar lower_red1(0, 120, 70);
        cv::Scalar upper_red1(10, 255, 255);

        // Create mask for red
        cv::Mat mask;
        cv::inRange(hsv, lower_red1, upper_red1, mask);

        // Compute centroid of the red mask
        cv::Moments M = cv::moments(mask);
        if (M.m00 > 0) {
            int box_center_x = static_cast<int>(M.m10 / M.m00);
            int img_width = cv_image.cols;
            int img_center_x = img_width / 2; // Image midpoint

            // Calculate yaw correction in radians, clamped to ±π/2
            double pixel_offset = box_center_x - img_center_x;
            double yaw_correction = (pixel_offset / img_width) * camera_fov_rad_;
            yaw_correction = std::clamp(yaw_correction, -max_yaw_correction_, max_yaw_correction_);

            // RCLCPP_INFO(this->get_logger(), "correction: %f", yaw_correction);

            // Compute new desired angle relative to current angle
            double desired_angle = current_angle_ - yaw_correction;
            publish_angle(desired_angle);
        } else {
            publish_angle(current_angle_ + .8); // No red detected → look around
        }

        // Display images
        cv::imshow("Original", cv_image);
        cv::imshow("Red Mask", mask);
        cv::waitKey(1); // Required to update the OpenCV window
    }

    void publish_angle(double angle) {
        // Publish desired angle
        auto msg = std_msgs::msg::Float32();
        msg.data = static_cast<float>(angle);
        anglepub_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Published angle: %.5f", angle);
    }

    // ROS2 Subscriptions and Publishers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imagesub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr anglesub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr anglepub_;

    // Constants and state variables
    double camera_fov_rad_;
    double max_yaw_correction_;
    double current_angle_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RedBoxAimer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}