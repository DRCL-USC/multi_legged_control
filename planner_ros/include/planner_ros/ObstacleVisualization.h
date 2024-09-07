
#include <ros/ros.h>
#include <ocs2_core/Types.h>
#include <visualization_msgs/MarkerArray.h>
#include <planner/ObjectInterface.h>
#include <std_msgs/Bool.h>
#include <ocs2_core/misc/LoadData.h>

class ObstacleVisualization
{
public:
    std::string frameId_ = "rod_odom"; // Frame name all messages are published in

    ObstacleVisualization(ros::NodeHandle &nodehandle, std::shared_ptr<ocs2::planner::Obstacles> obstaclesPtr, const std::string taskFile)
        : obstaclesPtr_(std::move(obstaclesPtr)), nh(nodehandle)
    {
        obstaclesPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("/obstacle_markers", 0);
        obstaclesSubscriber_ = nh.subscribe("/obstacle_visualizer", 1, &ObstacleVisualization::callback, this);

        ocs2::loadData::loadStdVector(taskFile, "obstacles.radius", obstacles_radius, false);
    }

    void callback(const std_msgs::Bool::ConstPtr &msg)
    {

        auto obstacles_pose = obstaclesPtr_->getObstacles();
        visualization_msgs::MarkerArray markerArray;
        ros::Time timeStamp = ros::Time::now();

        // Obstacle 1
        visualization_msgs::Marker marker;

        marker.header.frame_id = frameId_;
        marker.header.stamp = timeStamp;
        marker.ns = "obstacles";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 2.0;
        marker.scale.y = 2.0;
        marker.scale.z = 1.5;

        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        marker.pose.position.x = 3.6;
        marker.pose.position.y = -1.5;
        marker.pose.position.z = 0.75;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        markerArray.markers.push_back(marker);

        // Obstacle 2
        marker.id = 1;
        marker.scale.x = 0.3;
        marker.scale.y = 5.0;
        marker.scale.z = 2.0;

        marker.pose.position.x = 6.05;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 1.5;

        markerArray.markers.push_back(marker);

        // Obstacle 3
        marker.id = 2;
        marker.scale.x = 0.35;
        marker.scale.y = 0.4;
        marker.scale.z = 0.45;

        marker.pose.position.x = 8.6;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.2;

        markerArray.markers.push_back(marker);


        obstaclesPublisher_.publish(markerArray);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber obstaclesSubscriber_;
    ros::Publisher obstaclesPublisher_;
    std::shared_ptr<ocs2::planner::Obstacles> obstaclesPtr_;
    ocs2::scalar_array_t obstacles_radius;
};
