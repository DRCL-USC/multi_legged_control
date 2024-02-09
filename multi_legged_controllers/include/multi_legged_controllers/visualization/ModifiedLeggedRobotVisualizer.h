
#pragma once

#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>

namespace legged{

class ModifiedLeggedRobotVisualizer : public LeggedRobotVisualizer {
 public:
  ModifiedLeggedRobotVisualizer(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                        const PinocchioEndEffectorKinematics& endEffectorKinematics, ros::NodeHandle& nodeHandle,
                        scalar_t maxUpdateFrequency = 100.0);

  void launchVisualizerNode(ros::NodeHandle& nodeHandle);

 private:
  const CentroidalModelInfo centroidalModelInfo_;

  std::unique_ptr<robot_state_publisher::RobotStatePublisher> robotStatePublisherPtr_;

  ros::Publisher costDesiredBasePositionPublisher_;
  std::vector<ros::Publisher> costDesiredFeetPositionPublishers_;

  ros::Publisher stateOptimizedPublisher_;

  ros::Publisher currentStatePublisher_;

};

}  // namespace legged