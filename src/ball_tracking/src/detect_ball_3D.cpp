#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>

class DetectBall3d : public rclcpp::Node
{
    public:
        DetectBall3d() : Node("detect_ball_3d")
        {

        }

    private:
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto detect_ball_3d = std::make_shared<DetectBall3d>();
  rclcpp::spin(detect_ball_3d);
  rclcpp::shutdown();
  return 0;
}