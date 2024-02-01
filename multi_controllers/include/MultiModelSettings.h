
#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/ModelSettings.h>

namespace ocs2
{
  namespace legged_robot
  {

    struct MulltiModelSetting : public ModelSettings
    {
      MulltiModelSetting(const std::string &robotName)
      {
        jointNames = {robotName + "_LF_HAA", robotName + "_LF_HFE", robotName + "_LF_KFE", robotName + "_RF_HAA",
                      robotName + "_RF_HFE", robotName + "_RF_KFE", robotName + "_LH_HAA", robotName + "_LH_HFE",
                      robotName + "_LH_KFE", robotName + "_RH_HAA", robotName + "_RH_HFE", robotName + "_RH_KFE"};

        contactNames3DoF = {robotName + "_LF_foot", robotName + "_RF_foot", robotName + "_LH_foot", robotName + "_RH_foot"};

        std::cout << "MulltiModelSetting constructor\n";
      }
    };

  } // namespace legged_robot
} // namespace ocs2
