#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

#include "planner/definitions.h"
#include "planner_ros/StateEstimation.h"
#include <visualization_msgs/MarkerArray.h>

namespace ocs2
{
  namespace planner
  {

    class ObjectVisualization : public DummyObserver
    {
    public:
      /** Visualization settings (publicly available) */
      std::string frameId_ = "rod_odom";                                                                 // Frame name all messages are published in
      scalar_t trajectoryLineWidth_ = 0.01;                                                          // LineThickness for trajectories

      explicit ObjectVisualization(ros::NodeHandle &nodeHandle, const std::string taskfile);

      ~ObjectVisualization() override = default;

      void update(const SystemObservation &observation, const PrimalSolution &policy, const CommandData &command) override;

    private:
      void launchVisualizerNode(ros::NodeHandle &nodeHandle);
      void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t &mpcTimeTrajectory,
                                           const vector_array_t &mpcStateTrajectory);
      void publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories &targetTrajectories);
      visualization_msgs::Marker ObjectTrajectory(ros::Time timeStamp, const SystemObservation &observation);
      visualization_msgs::Marker ObjectTarget(ros::Time timeStamp, const CommandData &command);

      ros::Publisher desiredBasePositionPublisher_, stateOptimizedPublisher_, objectPublisher_, obstaclesPublisher_;
      const std::string taskFile_;
      StateEstimation stateEstimation;
    };

  } // namespace planner
} // namespace ocs2
