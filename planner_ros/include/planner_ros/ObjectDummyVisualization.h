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

    class ObjectDummyVisualization : public DummyObserver
    {
    public:
      /** Visualization settings (publicly available) */
      std::string frameId_ = "map";                                                                 // Frame name all messages are published in
      scalar_t trajectoryLineWidth_ = 0.01;                                                          // LineThickness for trajectories

      explicit ObjectDummyVisualization(ros::NodeHandle &nodeHandle, const std::string taskfile);

      ~ObjectDummyVisualization() override = default;

      void update(const SystemObservation &observation, const PrimalSolution &policy, const CommandData &command) override;

    private:
      void launchVisualizerNode(ros::NodeHandle &nodeHandle);
      void publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t &mpcTimeTrajectory,
                                           const vector_array_t &mpcStateTrajectory);
      void publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories &targetTrajectories);
      visualization_msgs::Marker ObjectTrajectory(ros::Time timeStamp, const SystemObservation &observation);
      visualization_msgs::Marker ObjectTarget(ros::Time timeStamp, const CommandData &command);

      ros::Publisher desiredBasePositionPublisher_;
      ros::Publisher stateOptimizedPublisher_;
      ros::Publisher objectPublisher_;
      ros::Publisher obstaclesPublisher_;
      const std::string taskFile_;
      StateEstimation stateEstimation;
    };

  } // namespace planner
} // namespace ocs2
