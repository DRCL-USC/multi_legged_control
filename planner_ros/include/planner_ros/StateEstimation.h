
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <ocs2_core/Types.h>
#include <planner/definitions.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <realtime_tools/realtime_buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
                state.setZero(STATE_DIM);
            };
        };

        class StateEstimation
        {
        public:
            StateEstimation(): tfListener(tfBuffer)
            {
                // Create a subscriber to the gazebo_model_state topic
                modelStateSub = nh.subscribe("/rod_state", 10, &StateEstimation::modelStateCallback, this);
            };

            void modelStateCallback(const nav_msgs::Odometry::ConstPtr &msg)
            {

                geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform("rod_odom", "map", ros::Time(0));

                object_data.time = msg->header.stamp.toSec();

                object_data.position(0) = msg->pose.pose.position.x + transform.transform.translation.x;
                object_data.position(1) = msg->pose.pose.position.y + transform.transform.translation.y;
                object_data.position(2) = msg->pose.pose.position.z + transform.transform.translation.z;

                object_data.quaternion = quaternion_t(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

                object_data.rotmat = object_data.quaternion.toRotationMatrix();
                object_data.rpy = object_data.rotmat.eulerAngles(0, 1, 2);

                object_data.v_world(0) = msg->twist.twist.linear.x;
                object_data.v_world(1) = msg->twist.twist.linear.y;
                object_data.v_world(2) = msg->twist.twist.linear.z;

                object_data.v_body = object_data.rotmat.transpose() * object_data.v_world;

                object_data.omega_world(0) = msg->twist.twist.angular.x;
                object_data.omega_world(1) = msg->twist.twist.angular.y;
                object_data.omega_world(2) = msg->twist.twist.angular.z;
                
                object_data.omega_body = object_data.rotmat.transpose() * object_data.omega_world;

                object_data.state << object_data.position,
                    object_data.quaternion.w(), object_data.quaternion.x(), object_data.quaternion.y(), object_data.quaternion.z(),
                    object_data.v_world, object_data.omega_world;

                publishBaseTransform(msg->header.stamp);    

                // ROS_INFO("I heard: x =%f, y=%f, yaw=%f , vx=%f, vy=%f, omega=%f",
                //          object_data.state[0], object_data.state[1], object_data.state[2], object_data.state[3], object_data.state[4], object_data.state[5]);
            };

            void publishBaseTransform(ros::Time time)
            {
                geometry_msgs::TransformStamped rodbase2odomTransform;
                rodbase2odomTransform.header.stamp = time;
                rodbase2odomTransform.header.frame_id = "rod_odom";
                rodbase2odomTransform.child_frame_id = "rod_base";

                rodbase2odomTransform.transform.translation.x = object_data.position(0);
                rodbase2odomTransform.transform.translation.y = object_data.position(1);
                rodbase2odomTransform.transform.translation.z = object_data.position(2);

                rodbase2odomTransform.transform.rotation.w = object_data.quaternion.w();
                rodbase2odomTransform.transform.rotation.x = object_data.quaternion.x();
                rodbase2odomTransform.transform.rotation.y = object_data.quaternion.y();
                rodbase2odomTransform.transform.rotation.z = object_data.quaternion.z();
                rodbase2odomTransform_broadcaster.sendTransform(rodbase2odomTransform);
            }

            StreamedData object_data;

        private:
            ros::NodeHandle nh;
            ros::Subscriber modelStateSub;
            tf::TransformBroadcaster rodbase2odomTransform_broadcaster;
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tfListener;
        };

    } // namespace planner
} // namespace ocs2
