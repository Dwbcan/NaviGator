#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
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
                                                                            std::bind(&DetectBall::object_detection_callback, this, std::placeholders::_1));

            // Create publisher to publish output image
            image_output_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_out", 1);
            
            // Create publisher to publish normalized centre point of detected ball
            ball_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/detected_ball", 1);

            // Create publisher to publish velocity commands that spins navibot in a circle until a ball is detected
            spin_velocity_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

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
        }

    private:
        void object_detection_callback(const sensor_msgs::msg::Image::SharedPtr data)
        {
            // Convert the ROS2 image message to OpenCV format using cv_bridge
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(data, "bgr8");
            cv::Mat cv_image = cv_ptr->image;


            // Detect circles in the image using the tuning parameters
            cv::Mat output_image, tuning_image;
            std::vector<cv::KeyPoint> keypoints_norm;
            std::tie(keypoints_norm, output_image, tuning_image) = find_circles(cv_image, this->tuning_params);


            // Convert the output image to ROS2 format and publish it
            sensor_msgs::msg::Image::SharedPtr img_to_pub = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", output_image).toImageMsg();
            img_to_pub->header = data->header;
            this->image_output_pub_->publish(*img_to_pub);


            // Find the largest circle and publish its normalized centre point position as a ROS2 message
            geometry_msgs::msg::Point point_out;
            point_out.z = 0.0;


            for(size_t i = 0; i < keypoints_norm.size(); ++i)
            {
              const auto& kp = keypoints_norm[i];
              const auto& x = kp.pt.x;
              const auto& y = kp.pt.y;
              const auto& s = kp.size;

                // Log the position and size of each circle
                RCLCPP_INFO(this->get_logger(), "Pt %d: (%f,%f,%f)", i, x, y, s);

                if(s > point_out.z)
                {
                  // Update the largest circle's centre point position
                  point_out.x = x;  // Please note the x value is the normalized x position of the centre point of the circle in terms of the fraction of the image frame it extends across (where the origin 0,0 is the centre of the image frame)
                  point_out.y = y;  // Please note the y value is the normalized y position of the centre point of the circle in terms of the fraction of the image frame it extends across (where the origin 0,0 is the centre of the image frame)
                  point_out.z = s;  // Please note the z value is the diameter of the circle in terms of the fraction of the image frame it extends across
                }
            }

            // Check if the circle has a valid diameter and if so, publish the normalized position of its centre point
            if(point_out.z > 0.0)
            {
              // Publish the largest circle position as a ROS2 message
              this->ball_pub_->publish(point_out);

              // Shutdown the node
              rclcpp::shutdown();
            }

            // Keep spinning robot until ball is detected
            else
            {
              geometry_msgs::msg::Twist spin_speed;
              spin_speed.angular.z = 0.6;
              this->spin_velocity_pub->publish(spin_speed);
            }
        }


        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_output_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr ball_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr spin_velocity_pub;
        std::map<std::string, int> tuning_params;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto detect_ball = std::make_shared<DetectBall>();
  while(rclcpp::ok())
  {
    rclcpp::spin_some(detect_ball);
    wait_on_gui();
  }
  rclcpp::shutdown();
  return 0;
}