#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

class FollowBall : public rclcpp::Node
{
    public:
        FollowBall() : Node("follow_ball")
        {
            // Create subscriber to localize navibot using Adaptive Monte Carlo Localization (AMCL)
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&FollowBall::amcl_callback, this, std::placeholders::_1));

            // Create subscriber that receives A* path from planned_path topic
            path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, std::bind(&FollowBall::path_callback, this, std::placeholders::_1));

            // Create publisher to publish command velocity and make navibot follow A* path
            cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        geometry_msgs::msg::PoseStamped robot_pose_;  // Navibot's current pose
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowBall>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}