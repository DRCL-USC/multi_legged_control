#include <ros/init.h>
#include <ros/package.h>

#include <GaitPublisher.h>

using namespace legged;

int main(int argc, char* argv[]) {
  const std::string robotName = argv[1];

  // Initialize ros node
  ros::init(argc, argv, robotName + "_mpc_mode_schedule");
  ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string gaitCommandFile;
  nodeHandle.getParam("/" + robotName + "gaitCommandFile", gaitCommandFile);
  std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;

  GaitPublisher gaitCommand(nodeHandle, gaitCommandFile, robotName, true);

  while (ros::ok() && ros::master::check()) {
    ros::spin();
  }

  // Successful exit
  return 0;
}