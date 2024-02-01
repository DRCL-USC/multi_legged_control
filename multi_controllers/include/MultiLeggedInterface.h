
#include <legged_interface/LeggedInterface.h>

namespace legged
{
  class MultiLeggedInterface : public legged::LeggedInterface
  {
  public:
  // using LeggedInterface::LeggedInterface;
    MultiLeggedInterface(const std::string robotName, const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                         bool useHardFrictionConeConstraint = false)
        : LeggedInterface(taskFile, urdfFile, referenceFile, useHardFrictionConeConstraint)
    {
      modelSettings_.jointNames = {robotName + "_LF_HAA", robotName + "_LF_HFE", robotName + "_LF_KFE", robotName + "_RF_HAA",
                                  robotName + "_RF_HFE", robotName + "_RF_KFE", robotName + "_LH_HAA", robotName + "_LH_HFE",
                                  robotName + "_LH_KFE", robotName + "_RH_HAA", robotName + "_RH_HFE", robotName + "_RH_KFE"};
      modelSettings_.contactNames3DoF = {robotName + "_LF_FOOT", robotName + "_RF_FOOT", robotName + "_LH_FOOT", robotName + "_RH_FOOT"};
    }
  };
} // namespace legged