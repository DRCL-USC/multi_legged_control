
#include <legged_controllers/LeggedController.h>
#include <multi_legged_controllers/visualization/FootPlacementVisualization.h>
#include <multi_legged_controllers/visualization/SphereVisualization.h>
#include <multi_legged_controllers/visualization/ModifiedLeggedRobotVisualizer.h>
#include <multi_legged_controllers/legged_estimation/ModifiedStateEstimateBase.h>

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
        void updateStateEstimation(const ros::Time &time, const ros::Duration &period) override;
        virtual void setupLeggedInterfaceWithNs(const std::string ns, const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                                  bool verbose);
        virtual void setupMPCwithNh(const std::string ns);

    private:
        benchmark::RepeatedTimer wbcTimer_;
        std::shared_ptr<ModifiedLeggedRobotVisualizer> ModifiedRobotVisualizer_;
        std::shared_ptr<ModifiedStateEstimateBase> stateEstimate_;
    };

    class MultiPerceptiveController : public MultiLeggedController
    {
    public:
        void update(const ros::Time &time, const ros::Duration &period) override;
    
    protected:
        void setupLeggedInterfaceWithNs(const std::string ns, const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                                  bool verbose) override;
        void setupMPCwithNh(const std::string ns) override;

    private:
        std::shared_ptr<FootPlacementVisualization> footPlacementVisualizationPtr_;
        std::shared_ptr<SphereVisualization> sphereVisualizationPtr_;
    };

} // namespace legged
