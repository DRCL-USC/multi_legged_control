#include "planner_ros/ObjectVisualization.h"
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <std_msgs/Bool.h>

namespace ocs2
{
  namespace planner
  {

    ObjectVisualization::ObjectVisualization(ros::NodeHandle &nodeHandle, const std::string taskfile) : taskFile_(taskfile)
    {
      launchVisualizerNode(nodeHandle);
    }

    void ObjectVisualization::update(const SystemObservation &observation, const PrimalSolution &policy, const CommandData &command)
    {

      const auto timeStamp = ros::Time::now();

      // Publish object trajectory
      visualization_msgs::MarkerArray objectmarkers;
      visualization_msgs::Marker target_marker = ObjectTarget(timeStamp, command);
      visualization_msgs::Marker object_marker = ObjectTrajectory(timeStamp, observation);
      objectmarkers.markers.push_back(target_marker);
      objectmarkers.markers.push_back(object_marker);
      objectPublisher_.publish(objectmarkers);

      // Publish desired trajectory
      publishDesiredTrajectory(timeStamp, command.mpcTargetTrajectories_);

      // Publish optimized trajectory
      publishOptimizedStateTrajectory(timeStamp, policy.timeTrajectory_, policy.stateTrajectory_);

      // Publish obstacles
      std_msgs::Bool msg;
      msg.data = true;
      obstaclesPublisher_.publish(msg);
    }

    void ObjectVisualization::launchVisualizerNode(ros::NodeHandle &nodeHandle)
    {
      desiredBasePositionPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/desiredTrajectory", 1);
      stateOptimizedPublisher_ = nodeHandle.advertise<visualization_msgs::Marker>("/optimizedStateTrajectory", 1);
      objectPublisher_ = nodeHandle.advertise<visualization_msgs::MarkerArray>("/object_markers", 0);
      obstaclesPublisher_ = nodeHandle.advertise<std_msgs::Bool>("/obstacle_visualizer", 0);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void ObjectVisualization::publishDesiredTrajectory(ros::Time timeStamp, const TargetTrajectories &targetTrajectories)
    {
      const auto &stateTrajectory = targetTrajectories.stateTrajectory;

      // Reserve com messages
      std::vector<geometry_msgs::Point> desiredBasePositionMsg;
      desiredBasePositionMsg.reserve(stateTrajectory.size());

      // std::cout << "stateTrajectory.size(): " << stateTrajectory.size() << std::endl;

      for (size_t j = 0; j < stateTrajectory.size(); j++)
      {
        const auto state = stateTrajectory.at(j);

        // Construct base pose msg
        geometry_msgs::Pose pose;
        vector_t basePose(3);
        basePose << state(0), state(1), state(2); // magic number
        pose.position = getPointMsg(basePose);

        // Fill message containers
        desiredBasePositionMsg.push_back(pose.position);
      }

      // std::cout << "desiredBasePositionMsg.size(): " << desiredBasePositionMsg.size() << std::endl;

      // Headers
      auto comLineMsg = getLineMsg(std::move(desiredBasePositionMsg), Color::yellow, trajectoryLineWidth_);
      comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
      comLineMsg.id = 0;

      // Publish
      desiredBasePositionPublisher_.publish(comLineMsg);
    }

    /******************************************************************************************************/
    /******************************************************************************************************/
    /******************************************************************************************************/
    void ObjectVisualization::publishOptimizedStateTrajectory(ros::Time timeStamp, const scalar_array_t &mpcTimeTrajectory,
                                                              const vector_array_t &mpcStateTrajectory)
    {
      if (mpcTimeTrajectory.empty() || mpcStateTrajectory.empty())
      {
        return; // Nothing to publish
      }

      // Reserve Com Msg
      std::vector<geometry_msgs::Point> mpcComPositionMsgs;
      mpcComPositionMsgs.reserve(mpcStateTrajectory.size());

      // Extract Com and Feet from state
      std::for_each(mpcStateTrajectory.begin(), mpcStateTrajectory.end(), [&](const vector_t &state)
                    {
        // Fill com position and pose msgs
        geometry_msgs::Pose pose;
        vector_t basePose(3);
        basePose << state(0), state(1), state(2); // magic number
        pose.position = getPointMsg(basePose);
        mpcComPositionMsgs.push_back(pose.position); });

      // Add headers and Id
      auto comLineMsg = getLineMsg(std::move(mpcComPositionMsgs), Color::green, trajectoryLineWidth_);
      comLineMsg.header = getHeaderMsg(frameId_, timeStamp);
      comLineMsg.id = 0;

      stateOptimizedPublisher_.publish(comLineMsg);
    }

    visualization_msgs::Marker ObjectVisualization::ObjectTrajectory(ros::Time timeStamp, const SystemObservation &observation)
    {
      // Marker visualization
      visualization_msgs::Marker marker;
      marker.header.frame_id = frameId_;
      marker.header.stamp = timeStamp;
      marker.ns = "object";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = stateEstimation.object_data.position(0);
      marker.pose.position.y = stateEstimation.object_data.position(1);
      marker.pose.position.z = stateEstimation.object_data.position(2);

      Eigen::Matrix<scalar_t, 3, 1> euler;
      euler << 0.0, 0.0, M_PI / 2;
      const Eigen::Quaternion<scalar_t> quat = getQuaternionFromEulerAnglesZyx(euler); // (yaw, pitch, roll
      const Eigen::Quaternion<scalar_t> target_quat = (stateEstimation.object_data.quaternion * quat).normalized();

      marker.pose.orientation.x = target_quat.x();
      marker.pose.orientation.y = target_quat.y();
      marker.pose.orientation.z = target_quat.z();
      marker.pose.orientation.w = target_quat.w();

      marker.scale.x = 0.06;
      marker.scale.y = 0.06;
      marker.scale.z = 1;

      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;

      return marker;
    }

    visualization_msgs::Marker ObjectVisualization::ObjectTarget(ros::Time timeStamp, const CommandData &command)
    {
      const auto &targetTrajectories = command.mpcTargetTrajectories_;

      // Marker visualization
      visualization_msgs::Marker marker;
      marker.header.frame_id = frameId_;
      marker.header.stamp = timeStamp;
      marker.ns = "target";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;

      marker.pose.position.x = targetTrajectories.stateTrajectory[1](0);
      marker.pose.position.y = targetTrajectories.stateTrajectory[1](1);
      marker.pose.position.z = targetTrajectories.stateTrajectory[1](2);

      Eigen::Matrix<scalar_t, 3, 1> euler;
      euler << 0.0, 0.0, M_PI / 2;
      const Eigen::Quaternion<scalar_t> quat = getQuaternionFromEulerAnglesZyx(euler); // (yaw, pitch, roll
      const Eigen::Quaternion<scalar_t> target_quat = (Eigen::Quaternion<scalar_t>(targetTrajectories.stateTrajectory[1](3), targetTrajectories.stateTrajectory[1](4), targetTrajectories.stateTrajectory[1](5), targetTrajectories.stateTrajectory[1](6)) * quat).normalized();

      marker.pose.orientation.x = target_quat.x();
      marker.pose.orientation.y = target_quat.y();
      marker.pose.orientation.z = target_quat.z();
      marker.pose.orientation.w = target_quat.w();

      marker.scale.x = 0.06;
      marker.scale.y = 0.06;
      marker.scale.z = 1;

      marker.color.a = 0.2; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;

      return marker;
    }

  } // namespace planner
} // namespace ocs2
