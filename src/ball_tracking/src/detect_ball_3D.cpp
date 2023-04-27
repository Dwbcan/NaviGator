#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

class DetectBall3D : public rclcpp::Node
{
    public:
        DetectBall3D() : Node("detect_ball_3D")
        {
            // Create subscriber to receive normalized position of detected ball's centre point
            ball2D_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
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

            h_fov_ = this->get_parameter("h_fov").as_double();
            v_fov_ = h_fov_ / this->get_parameter("aspect_ratio").as_double();
            ball_radius_ = this->get_parameter("ball_radius").as_double();
        }

    private:
        void pose_estimation_callback(const geometry_msgs::msg::Point::SharedPtr data)
        {
            // Calculate horizontal angle the ball's diameter covers on 2D image plane and Euclidean distance to ball's centre point
            double ang_size = data->z * h_fov_;
            double d = ball_radius_ / (tan(ang_size / 2));

            // Calculate vertical angle the ball's radius covers on 2D image plane, y value of ball's centre point (in 3D space), and projection of vector d (vector from camera to ball's centre) onto ground
            double y_ang = data->y * v_fov_ / 2;
            double y = d * sin(y_ang);
            double d_proj = d * cos(y_ang);

            // Calculate horizontal angle the ball's radius covers on 2D image plane and x and z values of ball's centre point (in 3D space)
            double x_ang = data->x * h_fov_ / 2;
            double x = d_proj * sin(x_ang);
            double z = d_proj * cos(x_ang);

            auto p = geometry_msgs::msg::Point();
            p.x = x;
            p.y = y;
            p.z = z;

            // Publish ball's 3D pose estimate
            ball3D_pub_->publish(p);
        }
        
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr ball2D_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball3D_pub_;
        double h_fov_;
        double v_fov_;
        double ball_radius_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto detect_ball_3D = std::make_shared<DetectBall3D>();
  rclcpp::spin(detect_ball_3D);
  rclcpp::shutdown();
  return 0;
}