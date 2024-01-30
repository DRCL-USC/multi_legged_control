
#include "MultiController.h"


#include <pluginlib/class_list_macros.hpp>

namespace legged {

bool AliengoController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh){
  
  LeggedController::init(robot_hw, controller_nh);



  return true;
}


}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::AliengoController, controller_interface::ControllerBase)
