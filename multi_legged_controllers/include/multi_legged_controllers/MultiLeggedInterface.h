
#include <legged_interface/LeggedInterface.h>

namespace legged
{
    using namespace ocs2;
    using namespace legged_robot;
    class MultiLeggedInterface : public LeggedInterface
    {
    public:
        MultiLeggedInterface(const std::string ns, const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                             bool useHardFrictionConeConstraint = false)
            : LeggedInterface(taskFile, urdfFile, referenceFile, useHardFrictionConeConstraint)
        {
            modelSettings_.contactNames3DoF = {ns + "_LF_FOOT", ns + "_RF_FOOT", ns + "_LH_FOOT", ns + "_RH_FOOT"};
            modelSettings_.jointNames = {ns + "_LF_HAA", ns + "_LF_HFE", ns + "_LF_KFE", ns + "_RF_HAA", ns + "_RF_HFE", ns + "_RF_KFE",
                                         ns + "_LH_HAA", ns + "_LH_HFE", ns + "_LH_KFE", ns + "_RH_HAA", ns + "_RH_HFE", ns + "_RH_KFE"};
        }
    };

} // namespace legged
