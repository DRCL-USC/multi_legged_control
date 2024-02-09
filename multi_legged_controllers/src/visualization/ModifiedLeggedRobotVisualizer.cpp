
#include "multi_legged_controllers/visualization/ModifiedLeggedRobotVisualizer.h"

// Additional messages not in the helpers file
#include <visualization_msgs/MarkerArray.h>

// URDF related
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>

namespace legged{

using namespace ocs2;
using namespace legged_robot;

ModifiedLeggedRobotVisualizer::ModifiedLeggedRobotVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                                             const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle,
                                             scalar_t maxUpdateFrequency)
    : LeggedRobotVisualizer(pinocchioInterface, centroidalModelInfo, endEffectorKinematics, nodeHandle, maxUpdateFrequency),
      centroidalModelInfo_(std::move(centroidalModelInfo)) {};

void ModifiedLeggedRobotVisualizer::launchVisualizerNode(ros::NodeHandle& nodeHandle) {
  costDesiredBasePositionPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("legged_robot/desiredBaseTrajectory", 1);
  costDesiredFeetPositionPublishers_.resize(centroidalModelInfo_.numThreeDofContacts);
  costDesiredFeetPositionPublishers_[0] = nodeHandle.advertise<visualization_msgs::Marker>("legged_robot/desiredFeetTrajectory/LF", 1);
  costDesiredFeetPositionPublishers_[1] = nodeHandle.advertise<visualization_msgs::Marker>("legged_robot/desiredFeetTrajectory/RF", 1);
  costDesiredFeetPositionPublishers_[2] = nodeHandle.advertise<visualization_msgs::Marker>("legged_robot/desiredFeetTrajectory/LH", 1);
  costDesiredFeetPositionPublishers_[3] = nodeHandle.advertise<visualization_msgs::Marker>("legged_robot/desiredFeetTrajectory/RH", 1);
  stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("legged_robot/optimizedStateTrajectory", 1);
  currentStatePublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("legged_robot/currentState", 1);

  // Load URDF model
  urdf::Model urdfModel;
  if (!urdfModel.initParamWithNodeHandle("legged_robot_description", nodeHandle)) {
    std::cerr << "[ModifiedLeggedRobotVisualizer] Could not read URDF from: \"legged_robot_description\"" << std::endl;
  } else {
    KDL::Tree kdlTree;
    kdl_parser::treeFromUrdfModel(urdfModel, kdlTree);

    robotStatePublisherPtr_.reset(new robot_state_publisher::RobotStatePublisher(kdlTree));
    robotStatePublisherPtr_->publishFixedTransforms(true);
  }
}

}  // namespace legged
