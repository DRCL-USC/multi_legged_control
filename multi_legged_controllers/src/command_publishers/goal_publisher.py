#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import sys

def publish_once(goal):
    # Initialize the ROS node
    rospy.init_node('goal_publisher_node', anonymous=True)

    # Create a publisher for the /robot_2/gait topic with std_msgs/String type
    pub1 = rospy.Publisher('/robot_1/goal', PoseStamped, queue_size=10)
    pub2 = rospy.Publisher('/robot_2/goal', PoseStamped, queue_size=10)

    # Wait for a short time to ensure the publisher is connected
    rospy.sleep(1)

    # Create the message
    msg = PoseStamped()
    msg.pose.position.x = float(goal)
    msg.pose.position.y = 0
    msg.pose.position.z = 0.4

    # Publish the message once
    pub1.publish(msg)
    pub2.publish(msg)
    rospy.loginfo("Published message: %s", msg.pose.position.x)

    # Optionally shut down the node after publishing
    rospy.signal_shutdown('Published message once')

if __name__ == '__main__':
    # Get the argument from the command line
    if len(sys.argv) < 2:
        print("Please provide the pose goal.")
        sys.exit(1)
    goal = sys.argv[1]

    # Call the publish_once function with the argument
    try:
        publish_once(goal)
    except rospy.ROSInterruptException:
        pass
