
#include <pinocchio/fwd.hpp>

#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <legged_wbc/WeightedWbc.h>
#include "multi_legged_controllers/PlanarTerrainReceiver.h"
#include "legged_perceptive_interface/PerceptiveLeggedReferenceManager.h"
#include "multi_legged_controllers/MultiLeggedInterface.h"
#include <multi_legged_controllers/legged_estimation/ModifiedLinearKalmanFilter.h>
#include "multi_legged_controllers/MultiLeggedController.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged {

bool MultiLeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh){
  
  std::string ns = controller_nh.getNamespace();
  // Extract the robot name from the namespace
  size_t stop = ns.find("/controllers");
  if (stop != std::string::npos) {
    ns = ns.substr(1,stop-1);
  }
  else {
    ROS_ERROR_STREAM("Could not extract robot name from namespace: " << ns);
    return false;
  }

  ros::NodeHandle nh(ns);
  
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  nh.getParam("urdfFile", urdfFile);
  nh.getParam("taskFile", taskFile);
  nh.getParam("referenceFile", referenceFile);
  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // setup the legged interface
  setupLeggedInterfaceWithNs(ns, taskFile, urdfFile, referenceFile, verbose);

  // Setup the MPC
  setupMPCwithNh(ns);

 // Setup MRT
  setupMrt();

  // Visualization
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  ModifiedRobotVisualizer_ = std::make_shared<ModifiedLeggedRobotVisualizer>(ns, leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
  //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{ ns + "_LF_HAA", ns + "_LF_HFE", ns + "_LF_KFE", ns + "_LH_HAA", ns + "_LH_HFE", ns + "_LH_KFE",
                                        ns + "_RF_HAA", ns + "_RF_HFE", ns + "_RF_KFE", ns + "_RH_HAA", ns + "_RH_HFE", ns + "_RH_KFE"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

  // State estimation
  stateEstimate_ = std::make_shared<ModifiedKalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, ns);
  dynamic_cast<ModifiedKalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);
  currentObservation_.time = 0;

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  return true;
}

void MultiLeggedController::setupLeggedInterfaceWithNs(const std::string ns, const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                                bool verbose) {
  leggedInterface_ = std::make_shared<MultiLeggedInterface>(ns, taskFile, urdfFile, referenceFile, verbose);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void MultiLeggedController::setupMPCwithNh(const std::string ns) {
  ros::NodeHandle nh(ns);
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robot_Name = "legged_robot";
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robot_Name);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robot_Name, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robot_Name + "_mpc_observation", 1);
}

void MultiLeggedController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  // Update the current state of the system
  mpcMrtInterface_->setCurrentObservation(currentObservation_);

  // Load the latest MPC policy
  mpcMrtInterface_->updatePolicy();

  // Evaluate the current policy
  vector_t optimizedState, optimizedInput;
  size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
  mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

  // Whole body control
  currentObservation_.input = optimizedInput;

  wbcTimer_.startTimer();
  vector_t x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
  wbcTimer_.endTimer();

  vector_t torque = x.tail(12);

  vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
  vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
  }

  // Visualization
  ModifiedRobotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  // selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

void MultiLeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  contact_flag_t contactFlag;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (size_t i = 0; i < contacts.size(); ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
  }
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

void MultiPerceptiveController::setupLeggedInterfaceWithNs(const std::string ns, const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                                bool verbose) {
  leggedInterface_ = std::make_shared<MultiPerceptiveLeggedInterface>(ns, taskFile, urdfFile, referenceFile, verbose);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void MultiPerceptiveController::setupMPCwithNh(const std::string ns) {
  MultiLeggedController::setupMPCwithNh(ns);
  ros::NodeHandle nh(ns);

  footPlacementVisualizationPtr_ = std::make_shared<FootPlacementVisualization>(ns,
      *dynamic_cast<PerceptiveLeggedReferenceManager&>(*leggedInterface_->getReferenceManagerPtr()).getConvexRegionSelectorPtr(),
      leggedInterface_->getCentroidalModelInfo().numThreeDofContacts, nh);

  sphereVisualizationPtr_ = std::make_shared<SphereVisualization>(ns,
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
      *dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPinocchioSphereInterfacePtr(), nh);

  // Planar terrain receiver
  auto planarTerrainReceiver =
      std::make_shared<PlanarTerrainReceiver>(nh, dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPlanarTerrainPtr(),
                                              dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getSignedDistanceFieldPtr(),
                                              "/convex_plane_decomposition_ros/planar_terrain", "elevation");
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

void MultiPerceptiveController::update(const ros::Time& time, const ros::Duration& period) {
  MultiLeggedController::update(time, period);
  footPlacementVisualizationPtr_->update(currentObservation_);
  sphereVisualizationPtr_->update(currentObservation_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::MultiLeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::MultiPerceptiveController, controller_interface::ControllerBase)
