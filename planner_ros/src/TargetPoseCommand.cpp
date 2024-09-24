#include <planner/definitions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>
#include <ros/package.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace ocs2;
using namespace planner;

namespace
{
  scalar_t targetDisplacementVelocity;
  scalar_t targetRotationVelocity;
} // namespace

scalar_t estimateTimeToTarget(const vector_t &desiredBaseDisplacement)
{
  const scalar_t &dx = desiredBaseDisplacement(0);
  const scalar_t &dy = desiredBaseDisplacement(1);
  // const scalar_t &dyaw = desiredBaseDisplacement(5);
  // const scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / targetDisplacementVelocity;
  // return std::max(rotationTime, displacementTime);
  return displacementTime;
}

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [deltaX, deltaY, deltaYaw]
 * @param [in] observation : the current observation
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t &commadLineTarget, const SystemObservation &observation)
{
  const vector_t currentPose = observation.state.segment<3>(0);
  const Eigen::Quaternion<scalar_t> currentQuaternion(observation.state(3), observation.state(4), observation.state(5), observation.state(6));

  Eigen::Matrix<scalar_t, 3, 1> euler;
  euler << commadLineTarget(5), commadLineTarget(4), commadLineTarget(3);
  euler *= M_PI / 180.0;
  const Eigen::Quaternion<scalar_t> quat_command =  getQuaternionFromEulerAnglesZyx(euler);

  const vector_t targetPose = [&]()
  {
    vector_t target(7);
    target.setZero();
    // p_x, p_y, and theta_z  are relative to current state
    target(0) = currentPose(0) + commadLineTarget(0);
    target(1) = currentPose(1) + commadLineTarget(1);
    target(2) = currentPose(2) + commadLineTarget(2);

    target(3) = quat_command.w();
    target(4) = quat_command.x();
    target(5) = quat_command.y();
    target(6) = quat_command.z();

    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(commadLineTarget.segment<3>(0));

  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << observation.state.segment<7>(0), vector_t::Zero(6);
  stateTrajectory[1] << targetPose, vector_t::Zero(6);

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char *argv[])
{
  const std::string robotName = "object";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  const std::string taskFile = ros::package::getPath("planner") + "/config/task.info";

  loadData::loadCppDataType(taskFile, "targetRotationVelocity", targetRotationVelocity);
  loadData::loadCppDataType(taskFile, "targetDisplacementVelocity", targetDisplacementVelocity);

  // goalPose: [deltaX, deltaY, deltaYaw]
  const scalar_array_t relativeBaseLimit{10.0, 10.0, 2.0, 180.0, 180.0, 180.0};
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(nodeHandle, robotName, relativeBaseLimit, &commandLineToTargetTrajectories);

  const std::string commandMsg = "Enter XYZ and Roll Pitch Yaw (deg) displacements for the object, separated by spaces";
  targetPoseCommand.publishKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}
