

#include <legged_controllers/LeggedController.h>

namespace legged
{
    using namespace ocs2;
    using namespace legged_robot;

    class MultiLeggedController : public legged::LeggedController
    {
    protected:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) override;
    };

} // namespace legged
