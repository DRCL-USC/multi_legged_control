#include <algorithm>
#include <iostream>
#include <ros/ros.h>
#include <ocs2_core/Types.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ros/package.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2
{
    namespace planner
    {
        class PlannerInterface
        {
        public:
            PlannerInterface(ros::NodeHandle &nodeHandle, const std::string &taskFile)
            {
                TargetTrajectoriesPublisher_[0].reset(new TargetTrajectoriesRosPublisher(nodeHandle, "/robot_1/legged_robot"));
                TargetTrajectoriesPublisher_[1].reset(new TargetTrajectoriesRosPublisher(nodeHandle, "/robot_2/legged_robot"));

                const std::string referenceFile = ros::package::getPath("legged_controllers") + "/config/aliengo/reference.info";
                DEFAULT_JOINT_STATE.resize(12);
                loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
                loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
                loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);

                robot_sub[0] = nodeHandle.subscribe<ocs2_msgs::mpc_observation>("/robot_1/legged_robot_mpc_observation", 1, &PlannerInterface::robot1Callback, this);
                robot_sub[1] = nodeHandle.subscribe<ocs2_msgs::mpc_observation>("/robot_2/legged_robot_mpc_observation", 1, &PlannerInterface::robot2Callback, this);
            }

            scalar_t estimateTimeToTarget(const vector_t &desiredBaseDisplacement)
            {
                const scalar_t &dx = desiredBaseDisplacement(0);
                const scalar_t &dy = desiredBaseDisplacement(1);
                const scalar_t &dz = desiredBaseDisplacement(2);
                const scalar_t &dyaw = desiredBaseDisplacement(5);
                const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
                const scalar_t displacement = std::sqrt(dx * dx + dy * dy + dz * dz);
                const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
                // return std::max(rotationTime, displacementTime);
                return displacementTime;
            }

            void publishTargetTrajectories(const SystemObservation &observation, const PrimalSolution &policy, const CommandData &command)
            {

                // Input trajectory
                const vector_array_t inputTrajectory(policy.timeTrajectory_.size(), vector_t::Zero(24));

                for (int i = 0; i < 2; i++)
                {
                    // Time trajectory
                    scalar_array_t timeTrajectory(policy.timeTrajectory_.size(), 0.0);
                    for (int j = 0; j < policy.timeTrajectory_.size(); j++)
                    {
                        timeTrajectory[j] = robot_observationPtr_[i].time + estimateTimeToTarget(policy.stateTrajectory_[j].segment<3>(0) - observation.state.segment<3>(0));
                    }

                    // State trajectory
                    vector_array_t stateTrajectory(policy.timeTrajectory_.size(), vector_t::Zero(24));
                    for (int j = 0; j < policy.timeTrajectory_.size(); j++)
                    {
                        const Eigen::Quaternion<scalar_t> quat = quaternion_t(policy.stateTrajectory_[j](3), policy.stateTrajectory_[j](4), policy.stateTrajectory_[j](5),
                                                                              policy.stateTrajectory_[j](6));
                        matrix3_t rotmat = quat.toRotationMatrix();

                        vector3_t robot_COM_target = policy.stateTrajectory_[j].segment<3>(0);
                        if (i == 0)
                        {
                            robot_COM_target += rotmat * (vector3_t() << 0.0, -0.5, 0.0).finished();
                            robot_COM_target += (vector3_t() << 0.0, 0.5, -0.1).finished();
                        }
                        else
                        {
                            robot_COM_target += rotmat * (vector3_t() << 0.0, 0.5, 0.0).finished();
                            robot_COM_target += (vector3_t() << 0.0, -0.5, -0.1).finished();
                        }
                        stateTrajectory[j] << vector_t::Zero(6), robot_COM_target(0), robot_COM_target(1), robot_COM_target(2),
                            vector_t::Zero(3), DEFAULT_JOINT_STATE; // roll, pitch and yaw are different
                    }

                    targetTrajectories_[i].timeTrajectory = timeTrajectory;
                    targetTrajectories_[i].stateTrajectory = stateTrajectory;
                    targetTrajectories_[i].inputTrajectory = inputTrajectory;

                    TargetTrajectoriesPublisher_[i]->publishTargetTrajectories(targetTrajectories_[i]);
                }
            };

            void robot1Callback(const ocs2_msgs::mpc_observation::ConstPtr &msg)
            {
                robot_observationPtr_[0] = ocs2::ros_msg_conversions::readObservationMsg(*msg);
            };

            void robot2Callback(const ocs2_msgs::mpc_observation::ConstPtr &msg)
            {
                robot_observationPtr_[1] = ocs2::ros_msg_conversions::readObservationMsg(*msg);
            };

        private:
            vector_t DEFAULT_JOINT_STATE;
            scalar_t TARGET_DISPLACEMENT_VELOCITY;
            scalar_t TARGET_ROTATION_VELOCITY;
            std::unique_ptr<TargetTrajectoriesRosPublisher> TargetTrajectoriesPublisher_[2];
            TargetTrajectories targetTrajectories_[2];
            ros::Subscriber robot_sub[2];
            ocs2::SystemObservation robot_observationPtr_[2];
        };

    } // namespace planner
} // namespace ocs2