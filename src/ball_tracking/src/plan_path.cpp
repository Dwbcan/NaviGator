#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <memory>

class PlanPath : public rclcpp::Node
{
    public:
        PlanPath() : rclcpp::Node("plan_path")
        {
            
        }
    private:
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlanPath>());
    rclcpp::shutdown();
    return 0;
}