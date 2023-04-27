#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>

class DetectBall3D : public rclcpp::Node
{
    public:
        DetectBall3D() : Node("detect_ball_3D")
        {
            // Create subscriber to receive normalized position of detected ball's centre point
            ball2d_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/detected_ball",
            10,
            std::bind(&DetectBall3D::pose_estimation_callback, this, std::placeholders::_1));

            // Create publisher to publish detected ball's 3D pose estimate (the position of the detected ball's centre point in 3D space)
            ball3D_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/detected_ball_3D",
            1);

            this->declare_parameter("h_fov", 1.089);
            this->declare_parameter("ball_radius", 0.033);
            this->declare_parameter("aspect_ratio", 4.0 / 3.0);

            auto h_fov_ = this->get_parameter("h_fov").as_double();
            auto aspect_ratio_ = this->get_parameter("aspect_ratio").as_double();
            auto ball_radius_ = this->get_parameter("ball_radius").as_double();
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ball2d_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball3D_pub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto detect_ball_3D = std::make_shared<DetectBall3D>();
  rclcpp::spin(detect_ball_3D);
  rclcpp::shutdown();
  return 0;
}