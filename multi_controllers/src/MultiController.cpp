
#include "MultiController.h"


#include <pluginlib/class_list_macros.hpp>

namespace legged {

bool AliengoController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh){
  
  LeggedController::init(robot_hw, controller_nh);



  return true;
}

bool A1Controller::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh){
  
  LeggedController::init(robot_hw, controller_nh);



  return true;
}


}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::AliengoController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::A1Controller, controller_interface::ControllerBase)
