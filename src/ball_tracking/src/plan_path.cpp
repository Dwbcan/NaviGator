#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/compute_path_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <memory>

class PlanPath : public rclcpp::Node
{
    public:
        PlanPath() : rclcpp::Node("plan_path")
        {
            costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap",
            rclcpp::QoS(rclcpp::KeepLast(10)),
            std::bind(&PlanPath::costmap_callback, this, std::placeholders::_1));

            nav2_client_ = this->create_client<nav2_msgs::srv::ComputePathToPose>("compute_path_to_pose");
        }
    private:
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber_;
        rclcpp::Client<nav2_msgs::srv::ComputePathToPose>::SharedPtr nav2_client_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanPath>());
    rclcpp::shutdown();
    return 0;
}