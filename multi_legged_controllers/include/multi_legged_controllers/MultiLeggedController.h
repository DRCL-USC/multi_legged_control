
#include <legged_controllers/LeggedController.h>
#include <legged_perceptive_controllers/visualization/FootPlacementVisualization.h>
#include <legged_perceptive_controllers/visualization/SphereVisualization.h>
#include <multi_legged_controllers/visualization/ModifiedLeggedRobotVisualizer.h>

namespace legged
{
    using namespace ocs2;
    using namespace legged_robot;

    class MultiLeggedController : public legged::LeggedController
    {
    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &controller_nh) override;
        void update(const ros::Time &time, const ros::Duration &period) override;

    protected:
        virtual void setupMPCwithNh(ros::NodeHandle nh);

    private:
        benchmark::RepeatedTimer wbcTimer_;
        std::shared_ptr<ModifiedLeggedRobotVisualizer> ModifiedRobotVisualizer_;
    };

    class MultiPerceptiveController : public MultiLeggedController
    {
    public:
        void update(const ros::Time &time, const ros::Duration &period) override;
    
    protected:
        void setupLeggedInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                                  bool verbose) override;
        void setupMPCwithNh(ros::NodeHandle nh) override;

    private:
        std::shared_ptr<FootPlacementVisualization> footPlacementVisualizationPtr_;
        std::shared_ptr<SphereVisualization> sphereVisualizationPtr_;
    };

} // namespace legged
