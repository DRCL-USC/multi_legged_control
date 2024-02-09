

#include <legged_controllers/LeggedController.h>

namespace legged
{
    using namespace ocs2;
    using namespace legged_robot;

    class MultiLeggedController : public legged::LeggedController
    {
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) override;
        std::shared_ptr<ModifiedLeggedRobotVisualizer> robotVisualizer_;
    };

} // namespace legged
