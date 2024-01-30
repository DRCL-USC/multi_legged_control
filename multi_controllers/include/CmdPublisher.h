
#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <ros/subscriber.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

namespace legged {
using namespace ocs2;

class CmdPublisher final {
 public:
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  CmdPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        tf2_(buffer_) {
    // Trajectories publisher
    CmdPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    // goal subscriber
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      try {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("Failure %s\n", ex.what());
        return;
      }

      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      CmdPublisher_->publishTargetTrajectories(trajectories);
    };

    // cmd_vel subscriber
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }

      if (mode == "PASSIVE") {
        vector_t cmdVel = vector_t::Zero(4); // x,y, yaw
        cmdVel[0] = msg->linear.x; 
        cmdVel[1] = msg->linear.y;
        cmdVel[2] = msg->linear.z;
        cmdVel[3] = msg->angular.z;

        const auto trajectories =
            cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
        CmdPublisher_->publishTargetTrajectories(trajectories);
        ROS_INFO("cmd_vel callback is running");
      }
    };

    auto modeCallback = [this](const std_msgs::String::ConstPtr& msg) {
      ROS_INFO_THROTTLE(5, "mode callback is running");
      mode = msg->data;
      // if (mode == "STAND" && high_cmd.mode == 6) {
      //   high_cmd.mode = 1;
      //   return;
      // }
      // Sport mode setting
      const vector_t currentPose = latestObservation_.state.segment<6>(6);
      vector_t cmdGoal = vector_t::Zero(6);
      std_msgs::String gait;
      if (mode == "PASSIVE") {
        gait.data = "trot";
        gaitpublisher.publish(gait);

      } else if (mode == "SIT") {
        gait.data = "stance";
        gaitpublisher.publish(gait);
        cmdGoal[0] = currentPose[0];
        cmdGoal[1] = currentPose[1];
        cmdGoal[2] = 0.05;
        cmdGoal[3] = currentPose[3];
        const auto trajectories =
            goalToTargetTrajectories_(cmdGoal, latestObservation_);
        CmdPublisher_->publishTargetTrajectories(trajectories);

      } else if (mode == "STAND") {
        gait.data = "stance";
        gaitpublisher.publish(gait);
        cmdGoal[0] = currentPose[0];
        cmdGoal[1] = currentPose[1];
        cmdGoal[2] = 0.45;
        cmdGoal[3] = currentPose[3];
        const auto trajectories =
            goalToTargetTrajectories_(cmdGoal, latestObservation_);
        CmdPublisher_->publishTargetTrajectories(trajectories);
      }
    };

    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, goalCallback);
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>(
        "/alien2/alien_driver/cmd_vel", 1, cmdVelCallback);
    gaitpublisher = nh.advertise<std_msgs::String>("/alien1/gait", 1, true);
    mode_sub = nh.subscribe<std_msgs::String>(
        "/alien2/spot_driver/mode", 1, modeCallback);
  }

 private:
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> CmdPublisher_;
  std::string mode;

  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_, mode_sub;
  ::ros::Publisher gaitpublisher;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace legged