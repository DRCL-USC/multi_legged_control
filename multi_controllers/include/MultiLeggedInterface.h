
#include <legged_interface/LeggedInterface.h>
#include "MultiModelSettings.h"

namespace legged
{
  using namespace ocs2;
  using namespace legged_robot;
  class MultiLeggedInterface : public LeggedInterface
  {
  public:
    MultiLeggedInterface(const std::string robotName, const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile,
                         bool useHardFrictionConeConstraint = false)
        : LeggedInterface(taskFile, urdfFile, referenceFile, useHardFrictionConeConstraint)
    {
      modelSettings_ = legged_robot::MulltiModelSetting(robotName);
    }

  protected:
    // Override the modelSettings_ member variable
    ModelSettings modelSettings_;
  };
} // namespace legged