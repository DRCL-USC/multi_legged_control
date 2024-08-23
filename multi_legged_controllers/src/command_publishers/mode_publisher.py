#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys

def publish_once(mode):
    # Initialize the ROS node
    rospy.init_node('mode_publisher_node', anonymous=True)

    # Create a publisher for the /robot_2/gait topic with std_msgs/String type
    pub1 = rospy.Publisher('/robot_1/mode', String, queue_size=10)
    pub2 = rospy.Publisher('/robot_2/mode', String, queue_size=10)

    # Wait for a short time to ensure the publisher is connected
    rospy.sleep(1)

    # Create the message
    msg = String()
    msg.data = mode

    # Publish the message once
    pub1.publish(msg)
    pub2.publish(msg)
    rospy.loginfo("Published message: %s", msg.data)

    # Optionally shut down the node after publishing
    rospy.signal_shutdown('Published message once')

if __name__ == '__main__':
    # Get the argument from the command line
    if len(sys.argv) < 2:
        print("Please provide the mode type.")
        sys.exit(1)
    mode = sys.argv[1]

    # Call the publish_once function with the argument
    try:
        publish_once(mode)
    except rospy.ROSInterruptException:
        pass
