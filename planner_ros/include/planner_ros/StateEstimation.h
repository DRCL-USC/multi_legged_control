
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ocs2_core/Types.h>

namespace ocs2
{
    namespace planner
    {

        using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;
        using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;
        using quaternion_t = Eigen::Quaternion<scalar_t>;

        struct StreamedData
        {
            scalar_t time;
            vector3_t position;
            quaternion_t quaternion;
            matrix3_t rotmat;
            vector3_t rpy;
            vector3_t v_world;
            vector3_t v_body;
            vector3_t omega_world;
            vector3_t omega_body;
            vector_t state;
            StreamedData()
            {
                time = 0.0;
                position.setZero();
                quaternion = quaternion_t(1, 0, 0, 0);
                rotmat.setIdentity();
                rpy.setZero();
                v_world.setZero();
                v_body.setZero();
                omega_world.setZero();
                omega_body.setZero();
                state.setZero(12);
            };
        };

        class StateEstimation
        {
        public:
            StateEstimation()
            {
                // Create a subscriber to the gazebo_model_state topic
                modelStateSub = nh.subscribe("/rod_state", 10, &StateEstimation::modelStateCallback, this);
            };

            void modelStateCallback(const nav_msgs::Odometry::ConstPtr &msg)
            {

                object_data.time = msg->header.stamp.toSec();

                object_data.position(0) = msg->pose.pose.position.x;
                object_data.position(1) = msg->pose.pose.position.y;
                object_data.position(2) = msg->pose.pose.position.z;

                object_data.quaternion = quaternion_t(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

                object_data.rotmat = object_data.quaternion.toRotationMatrix();
                object_data.rpy = object_data.rotmat.eulerAngles(0, 1, 2);

                object_data.v_world(0) = msg->twist.twist.linear.x;
                object_data.v_world(1) = msg->twist.twist.linear.y;
                object_data.v_world(2) = msg->twist.twist.linear.z;

                object_data.v_body = object_data.rotmat * object_data.v_world;

                object_data.omega_world(0) = msg->twist.twist.angular.x;
                object_data.omega_world(1) = msg->twist.twist.angular.y;
                object_data.omega_world(2) = msg->twist.twist.angular.z;
                object_data.omega_body = object_data.rotmat * object_data.omega_world;

                object_data.state << object_data.position(0), object_data.position(1), object_data.position(2),
                    object_data.rpy(0), object_data.rpy(1), object_data.rpy(2),
                    object_data.v_world(0), object_data.v_world(1), object_data.v_world(2),
                    object_data.omega_world(0), object_data.omega_world(1), object_data.omega_world(2);

                // ROS_INFO("I heard: x =%f, y=%f, yaw=%f , vx=%f, vy=%f, omega=%f",
                //          object_data.state[0], object_data.state[1], object_data.state[2], object_data.state[3], object_data.state[4], object_data.state[5]);
            };

            StreamedData object_data;

        private:
            ros::NodeHandle nh;
            ros::Subscriber modelStateSub;
        };

    } // namespace planner
} // namespace ocs2
