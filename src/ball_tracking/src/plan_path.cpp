#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/srv/compute_path_to_pose.hpp"
#include "nav2_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <memory>

class PlanPath : public rclcpp::Node
{
    public:
        PlanPath() : rclcpp::Node("plan_path")
        {
            // Create subscriber to localize navibot using Adaptive Monte Carlo Localization (AMCL)
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&PlanPath::amcl_callback, this, std::placeholders::_1));

            // Create subscriber to receive 3D pose estimate of ball in map frame
            ball3D_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/detected_ball_3D", 1, std::bind(&PlanPath::ball3D_callback, this, std::placeholders::_1));

            // Create subscriber to receive costmap for A* path planning
            costmap_sub_ = this->create_subscription<nav2_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 10),
            std::bind(&PlanPath::costmap_callback, this, std::placeholders::_1));

            // Create client to send request to compute_path_to_pose service and receive planned path response
            nav2_client_ = this->create_client<nav2_msgs::srv::ComputePathToPose>("compute_path_to_pose");

            // Create publisher to publish planned A* path
            path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        }
    private:
        void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            // Log navibot's current pose
            RCLCPP_INFO(this->get_logger(), "Received robot pose: x=%f, y=%f, z=%f", msg->pose.pose.position.x,
            msg->pose.pose.position.y, msg->pose.pose.position.z);

            // Store navibot's current pose as a PoseStamped message
            robot_pose_.header = msg->header;
            robot_pose_.pose = msg->pose.pose;
        }

        void ball3D_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
        {
            ball3D_position_ = *msg;
        }

        void costmap_callback(const std::shared_ptr<nav2_msgs::msg::OccupancyGrid> msg)
        {
            // Wait for the compute_path_to_pose service to become available
            while(!nav2_client_->wait_for_service(std::chrono::seconds(1)))
            {
                // If the service doesn't become available within 1 second, check if the program is still running
                if(!rclcpp::ok())
                {
                    // If the program has been interrupted, log an error and exit the function
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                // If the program is still running, log a message indicating that we're still waiting for the service
                RCLCPP_INFO(this->get_logger(), "Waiting for service...");
            }

            // Create a request for the compute_path_to_pose service
            std::shared_ptr<nav2_msgs::srv::ComputePathToPose::Request> req = std::make_shared<nav2_msgs::srv::ComputePathToPose::Request>();

            // Set the start pose to navibot's current pose in the map frame
            req->start = robot_pose_;

            // Set the goal pose to the PointStamped message received from the 3D pose estimation node
            req->goal.header.frame_id = msg->header.frame_id;
            req->goal.pose.position = ball3D_position_.point;
            req->goal.pose.orientation.w = 1.0;

            // Set the planner ID to "GridBased"
            req->planner_id = "GridBased";

            // Send the request to the compute_path_to_pose service and wait for a response
            auto future_result = nav2_client_->async_send_request(req);
            auto result = future_result.get();

            // Check if the path planning was successful
            if(result->error_code != result->SUCCESS)
            {
                // If the path planning failed, log an error with the error code
                RCLCPP_ERROR(this->get_logger(), "Failed to plan path: %d", result->error_code);
            } 
            else
            {
                // If the path planning was successful, log the number of poses in the path
                RCLCPP_INFO(this->get_logger(), "Planned path with %d poses", result->path.poses.size());
            }
        }

        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr ball3D_sub_;
        rclcpp::Subscription<nav2_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
        rclcpp::Client<nav2_msgs::srv::ComputePathToPose>::SharedPtr nav2_client_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
        geometry_msgs::msg::PoseStamped robot_pose_;  // Navibot's current pose
        geometry_msgs::msg::PointStamped ball3D_position_;  // 3D pose estimate of ball
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanPath>());
    rclcpp::shutdown();
    return 0;
}