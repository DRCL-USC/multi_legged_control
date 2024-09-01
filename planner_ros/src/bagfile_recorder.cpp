#include <ros/ros.h>
#include <rosbag/bag.h>
#include <boost/shared_ptr.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <boost/filesystem.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <tf2_msgs/TFMessage.h>

#include <ocs2_msgs/mpc_observation.h>
#include <grid_map_msgs/GridMap.h>

std::string getCurrentDateTime()
{
    auto now = std::chrono::system_clock::now();
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&currentTime), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

class TopicRecorder
{
public:
    TopicRecorder(ros::NodeHandle &nh, const std::string &bagFileName)
        : nh_(nh), bag_(bagFileName, rosbag::bagmode::Write)
    {
    }

    template <typename M>
    void subscribe(const std::string &topic)
    {
        subscribers_.push_back(nh_.subscribe<M>(topic, 1000, [this, topic](const boost::shared_ptr<M const> &msg)
                                                { ros::Time timeStamp = ros::Time::now();
                                                    if (timeStamp.toNSec() == 0) timeStamp = ros::TIME_MIN;
                                                    this->bag_.write(topic, timeStamp, *msg); }));
    }

    ~TopicRecorder()
    {
        bag_.close();
    }

private:
    ros::NodeHandle nh_;
    rosbag::Bag bag_;
    std::vector<ros::Subscriber> subscribers_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "topic_recorder");
    ros::NodeHandle nh;

    std::string folderName = "/home/mohsen/Projects/ocs2_ws/bagfiles";

    // Create the folder if it does not exist
    boost::filesystem::path folderPath(folderName);
    if (!boost::filesystem::exists(folderPath))
    {
        boost::filesystem::create_directory(folderPath);
    }

    std::string bagFileName = folderName + "/recorded_topics_" + getCurrentDateTime() + ".bag";

    TopicRecorder recorder(nh, bagFileName);

    // palnner
    recorder.subscribe<ocs2_msgs::mpc_observation>("/object_mpc_observation");
    recorder.subscribe<visualization_msgs::MarkerArray>("/object_markers");
    recorder.subscribe<visualization_msgs::MarkerArray>("/obstacle_markers");
    recorder.subscribe<visualization_msgs::Marker>("/optimizedStateTrajectory");
    recorder.subscribe<visualization_msgs::Marker>("/desiredTrajectory");

    // elevation_mapping
    recorder.subscribe<grid_map_msgs::GridMap>("/elevation_mapping/elevation_map");
    recorder.subscribe<visualization_msgs::MarkerArray>("/convex_plane_decomposition_ros/boundaries");

    // tf
    recorder.subscribe<tf2_msgs::TFMessage>("/tf");
    recorder.subscribe<tf2_msgs::TFMessage>("/tf_static");

    std::vector<std::string> robotNamespaces = {"/robot_1", "/robot_2"};

    for (const auto &robotNamespace : robotNamespaces)
    {
        recorder.subscribe<visualization_msgs::MarkerArray>(robotNamespace + "/foot_placement");
        recorder.subscribe<visualization_msgs::MarkerArray>(robotNamespace + "/legged_robot/currentState");
        recorder.subscribe<visualization_msgs::MarkerArray>(robotNamespace + "/legged_robot/optimizedStateTrajectory");
        recorder.subscribe<visualization_msgs::Marker>(robotNamespace + "/legged_robot/desiredBaseTrajectory");
    }

    ros::spin();
    return 0;
}
