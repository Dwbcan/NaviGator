#include "rclcpp/rclcpp.hpp"
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
            // Create subscriber to receive costmap for A* path planning
            costmap_sub_ = this->create_subscription<nav2_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", 10),
            std::bind(&PlanPath::costmap_callback, this, std::placeholders::_1));

            // Create subscriber to localize navibot using Adaptive Monte Carlo Localization (AMCL)
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&PlanPath::amcl_callback, this, std::placeholders::_1));

            // Create client to send request to ComputePathToPose service and receive planned path response
            nav2_client_ = this->create_client<nav2_msgs::srv::ComputePathToPose>("compute_path_to_pose");

            // Create publisher to published planned A* path
            path_pub = this->create_publisher<nav2_msgs::msg::Path>("/planned_path", 10);
        }
    private:
        void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            // Log navibot's current pose
            RCLCPP_INFO(this->get_logger(), "Received robot pose: x=%f, y=%f, z=%f", msg->pose.pose.position.x,
                        msg->pose.pose.position.y, msg->pose.pose.position.z);
        }

        rclcpp::Subscription<nav2_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
        rclcpp::Client<nav2_msgs::srv::ComputePathToPose>::SharedPtr nav2_client_;
        rclcpp::Publisher<nav2_msgs::msg::Path>::SharedPtr path_pub;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanPath>());
    rclcpp::shutdown();
    return 0;
}