#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h> // Include the header file for gazebo_msgs::SetModelState

int main(int argc, char** argv)
{
    // Initialize the ROS node
   

    ros::init(argc, argv, "move_box");
    ros::NodeHandle nh;

    // Your code here
    // Create a service client to call the gazebo/set_model_state service
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    // Create a SetModelState request message
    gazebo_msgs::SetModelState srv;
    srv.request.model_state.model_name = "box";
    srv.request.model_state.pose.position.x = 0.0;
    srv.request.model_state.pose.position.y = 0.5;
    srv.request.model_state.pose.position.z = 0.5;

    // Call the service to set the model state
    if (client.call(srv))
    {
        ROS_INFO("Successfully set the position of the box model in Gazebo.");
    }
    else
    {
        ROS_ERROR("Failed to set the position of the box model in Gazebo.");
    }

    return 0;
}