#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include "process_image.hpp"

class DetectBall : public rclcpp::Node
{
public:
  DetectBall()
  : Node("detect_ball_2D")
  {

  }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectBall>());
    rclcpp::shutdown();
    return 0;
}