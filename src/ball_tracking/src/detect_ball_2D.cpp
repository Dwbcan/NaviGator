#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cv_bridge/cv_bridge.h>
#include "process_image.hpp"

class DetectBall : public rclcpp::Node
{
    public:
        DetectBall() : Node("detect_ball_2D")
        {
            RCLCPP_INFO(this->get_logger(), "Looking for the ball...");

            // Create subscriber to receive RGB camera images from /camera/image_raw topic
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw",
                                                                            rclcpp::SensorDataQoS(),
                                                                            std::bind(&DetectBall::callback, this, std::placeholders::_1));

            // Create publisher to publish output image
            image_output_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_out", 1);
            
            // Create publisher to publish tuning image
            image_tuning_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_tuning", 1);
            
            // Create publisher to publish normalized centre point of detected ball
            ball_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/detected_ball", 1);

            // Declare parameters
            this->declare_parameter("x_min", 0);
            this->declare_parameter("x_max", 0);
            this->declare_parameter("y_min", 0);
            this->declare_parameter("y_max", 0);
            this->declare_parameter("h_min", 0);
            this->declare_parameter("h_max", 0);
            this->declare_parameter("s_min", 0);
            this->declare_parameter("s_max", 0);
            this->declare_parameter("v_min", 0);
            this->declare_parameter("v_max", 0);
            this->declare_parameter("sz_min", 0);
            this->declare_parameter("sz_max", 0);

            auto x_min = this->get_parameter("x_min").as_int();
            auto x_max = this->get_parameter("x_max").as_int();
            auto y_min = this->get_parameter("y_min").as_int();
            auto y_max = this->get_parameter("y_max").as_int();
            auto h_min = this->get_parameter("h_min").as_int();
            auto h_max = this->get_parameter("h_max").as_int();
            auto s_min = this->get_parameter("s_min").as_int();
            auto s_max = this->get_parameter("s_max").as_int();
            auto v_min = this->get_parameter("v_min").as_int();
            auto v_max = this->get_parameter("v_max").as_int();
            auto sz_min = this->get_parameter("sz_min").as_int();
            auto sz_max = this->get_parameter("sz_max").as_int();

            std::map<std::string, int> tuning_params = {
                {"x_min", x_min},
                {"x_max", x_max},
                {"y_min", y_min},
                {"y_max", y_max},
                {"h_min", h_min},
                {"h_max", h_max},
                {"s_min", s_min},
                {"s_max", s_max},
                {"v_min", v_min},
                {"v_max", v_max},
                {"sz_min", sz_min},
                {"sz_max", sz_max}
            };

            cv_bridge::CvImage cv_ptr;

            create_tuning_window(tuning_params);
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_output_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_tuning_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_pub_;
        std::map<std::string, int> tuning_params_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto detect_ball = std::make_shared<DetectBall>();
  while (rclcpp::ok())
  {
    rclcpp::spin_some(detect_ball);
    wait_on_gui();
  }
  rclcpp::shutdown();
  return 0;
}