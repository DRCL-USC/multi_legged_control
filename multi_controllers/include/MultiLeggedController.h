

#include <legged_controllers/LeggedController.h>
#include "MultiLeggedInterface.h"

namespace legged
{
    using namespace ocs2;
    using namespace legged_robot;

    class MultiLeggedController : public legged::LeggedController
    {
    protected:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) override;
        void setupMpc() override;
        void update(const ros::Time &time, const ros::Duration &period) override;                          

    private:
        std::string robotName;
    };

} // namespace legged
