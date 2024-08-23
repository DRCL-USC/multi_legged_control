#include <ros/init.h>
#include <ros/package.h>
#include <iostream>

#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <planner/ObjectInterface.h>
#include <planner/definitions.h>

#include "planner_ros/ObjectVisualization.h"
#include <ocs2_mpc/SystemObservation.h>

using namespace ocs2;

int main(int argc, char **argv)
{
  const std::string robotName = "object";

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mrt");
  ros::NodeHandle nodeHandle;

  // Robot interface
  const std::string taskFile = ros::package::getPath("planner") + "/config/task.info";
  const std::string libFolder = ros::package::getPath("planner") + "/auto_generated";
  ocs2::planner::ObjectInterface objectInterface(taskFile, libFolder, false /*verbose*/);

  // State Estimation
  ocs2::planner::StateEstimation stateEstimation;

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  mrt.initRollout(&objectInterface.getRollout());
  mrt.launchNodes(nodeHandle);

  // Visualization
  auto ObjectVisualization = std::make_shared<ocs2::planner::ObjectVisualization>(nodeHandle, taskFile);

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state = stateEstimation.object_data.state;
  initObservation.input.setZero(ocs2::planner::INPUT_DIM);
  initObservation.time = 0.0;

  // initial command
  const ocs2::TargetTrajectories initTargetTrajectories({0.0, initObservation.time}, {ocs2::vector_t::Zero(ocs2::planner::STATE_DIM), initObservation.state},
                                                        {ocs2::vector_t::Zero(ocs2::planner::INPUT_DIM), initObservation.input});

  // Run MRT loop

  ROS_INFO_STREAM("Waiting for the initial policy ...");

  // Reset MPC node
  mrt.resetMpcNode(initTargetTrajectories);

  // Wait for the initial policy
  while (!mrt.initialPolicyReceived() && ros::ok() && ros::master::check())
  {
    mrt.spinMRT();
    mrt.setCurrentObservation(initObservation);
    ros::Rate(objectInterface.mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  const auto mpcUpdateRatio = std::max(static_cast<size_t>(objectInterface.mpcSettings().mrtDesiredFrequency_ / objectInterface.mpcSettings().mpcDesiredFrequency_), size_t(1));

  // Loop variables
  size_t loopCounter = 0;
  ocs2::SystemObservation currentObservation = initObservation;

  // Helper function to check if policy is updated and starts at the given time.
  // Due to ROS message conversion delay and very fast MPC loop, we might get an old policy instead of the latest one.
  const auto policyUpdatedForTime = [&](scalar_t time)
  {
    constexpr scalar_t tol = 0.1; // policy must start within this fraction of dt
    return mrt.updatePolicy() && std::abs(mrt.getPolicy().timeTrajectory_.front() - time) < (tol / objectInterface.mpcSettings().mpcDesiredFrequency_);
  };

  ros::Rate simRate(objectInterface.mpcSettings().mrtDesiredFrequency_);

  while (ros::ok())
  {
    // std::cout << "### Current time " << currentObservation.time << "\n";

    // Trigger MRT callbacks
    mrt.spinMRT();

    // Update the MPC policy if it is time to do so
    if (loopCounter % mpcUpdateRatio == 0)
    {
      // Wait for the policy to be updated
      while (!policyUpdatedForTime(currentObservation.time) && ros::ok() && ros::master::check())
      {
        mrt.spinMRT();
      }
      // std::cout << "<<< New MPC policy starting at " << mrt.getPolicy().timeTrajectory_.front() << "\n";
    }

    const scalar_t dt = 1.0 / objectInterface.mpcSettings().mrtDesiredFrequency_;

    SystemObservation nextObservation;
    nextObservation.time = currentObservation.time + dt;
    if (mrt.isRolloutSet())
    { // If available, use the provided rollout as to integrate the dynamics.
      mrt.rolloutPolicy(currentObservation.time, currentObservation.state, dt, nextObservation.state, nextObservation.input,
                        nextObservation.mode);
    }
    else
    { // Otherwise, we fake integration by interpolating the current MPC policy at t+dt
      mrt.evaluatePolicy(currentObservation.time + dt, currentObservation.state, nextObservation.state, nextObservation.input,
                         nextObservation.mode);
    }

    // Update MPC observation;
    currentObservation.time = nextObservation.time;
    currentObservation.input = nextObservation.input;
    currentObservation.state = stateEstimation.object_data.state;

    // Publish observation if at the next step we want a new policy
    if ((loopCounter + 1) % mpcUpdateRatio == 0)
    {
      mrt.setCurrentObservation(currentObservation);
      // std::cout << ">>> Observation is published at " << currentObservation.time << "\n";
    }

    // update Visualizer
    ObjectVisualization->update(currentObservation, mrt.getPolicy(), mrt.getCommand());

    ++loopCounter;
    ros::spinOnce();
    simRate.sleep();
  }

  return 0;
}
