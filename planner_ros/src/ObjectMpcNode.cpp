#include <memory>

#include <ros/init.h>
#include <ros/package.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/synchronized_module/SolverObserverRosCallbacks.h>
#include <planner/ObjectInterface.h>

int main(int argc, char** argv) {
  const std::string robotName = "object";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc");
  ros::NodeHandle nodeHandle;

  // Robot interface
  const std::string taskFile = ros::package::getPath("planner") + "/config/task.info";
  const std::string libFolder = ros::package::getPath("planner") + "/auto_generated";
  ocs2::planner::ObjectInterface objectInterface(taskFile, libFolder, true /*verbose*/);
  
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, objectInterface.getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // MPC
  ocs2::GaussNewtonDDP_MPC mpc(objectInterface.mpcSettings(), objectInterface.ddpSettings(),
                               objectInterface.getRollout(), objectInterface.getOptimalControlProblem(),
                               objectInterface.getInitializer());                           
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);

  // add adaptive control
  // auto adaptivecontrolPtr = objectInterface.getAdaptiveControlPtr();
  // mpc.getSolverPtr()->addSynchronizedModule(adaptivecontrolPtr);

  //add obstacles
  // auto obstaclesPtr = objectInterface.getObstaclesPtr();
  // mpc.getSolverPtr()->addSynchronizedModule(obstaclesPtr);

  // add visualization
  // ObstacleVisualization obstacleVisualization(nodeHandle, obstaclesPtr);

  // Launch MPC ROS node
  ocs2::MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);

  return 0;
}
