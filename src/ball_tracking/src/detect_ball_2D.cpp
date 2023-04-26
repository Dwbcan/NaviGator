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