#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
#include <cmath>

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
        void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            // Store navibot's current pose as a PoseStamped message
            robot_pose_.header = msg->header;
            robot_pose_.pose = msg->pose.pose;
        }

        void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
        {
            // Stop navibot if planned path is invalid (has no poses in path)
            if(msg->poses.size() < 1)
            {
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel_pub_->publish(cmd_vel);
                RCLCPP_INFO(this->get_logger(), "Invalid path!");
                return;
            }

            // Calculate distance from navibot to goal pose
            double distance = std::sqrt(std::pow(msg->poses.back().pose.position.x - robot_pose_.pose.position.x, 2) +
            std::pow(msg->poses.back().pose.position.y - robot_pose_.pose.position.y, 2));

            // Stop navibot if goal pose has been reached (if navibot's current pose is within 0.1 m from goal pose)
            if(distance < 0.1)
            {
                geometry_msgs::msg::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel_pub_->publish(cmd_vel);
                return;
            }

            // Get navibot's current yaw
            tf2::Quaternion quat;
            tf2::fromMsg(robot_pose_.pose.orientation, quat);  // Convert orientation of navibot's current pose to tf2::Quaternion object
            double current_yaw = tf2::getYaw(quat);

            // Calculate angle to next pose in path
            double goal_angle = std::atan2(msg->poses[1].pose.position.y - robot_pose_.pose.position.y, 
            msg->poses[1].pose.position.x - robot_pose_.pose.position.x) - current_yaw;

            // Normalize angle to be within -pi to pi
            while(goal_angle > M_PI)
            {
                goal_angle -= 2 * M_PI;
            }
            while(goal_angle < -M_PI)
            {
                goal_angle += 2 * M_PI;
            }

            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular = goal_angle;  // This is set to rotate navibot at a rate of goal_angle radians per second
            
            // Start 1 second timer
            auto start_time = std::chrono::high_resolution_clock::now();

            // Rotate navibot at a rate of goal_angle radians per second for 1 second (so navibot rotates by goal_angle radians)
            while(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() < 1)
            {
                cmd_vel_pub_->publish(cmd_vel);
            }

            // Stop navibot after rotation
            cmd_vel.angular = 0;
            cmd_vel_pub_->publish(cmd_vel);

            // Move navibot forward in direction it's currently facing
            cmd_vel.linear.x = 0.6;
            cmd_vel_pub_->publish(cmd_vel);
        }

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