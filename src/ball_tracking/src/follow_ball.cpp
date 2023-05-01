#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

class FollowBall : public rclcpp::Node
{
    public:
        FollowBall() : Node("follow_ball")
        {
            
        }

    private:
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowBall>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}