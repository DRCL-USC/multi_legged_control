
#include <legged_controllers/LeggedController.h>
#include <multi_legged_controllers/visualization/ModifiedLeggedRobotVisualizer.h>

namespace legged
{
    using namespace ocs2;
    using namespace legged_robot;

    class MultiLeggedController : public legged::LeggedController
    {
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) override;
        void update(const ros::Time &time, const ros::Duration &period) override;
        benchmark::RepeatedTimer wbcTimer_;
        std::shared_ptr<ModifiedLeggedRobotVisualizer> ModifiedRobotVisualizer_;
    };

} // namespace legged
