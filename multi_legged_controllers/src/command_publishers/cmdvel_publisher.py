#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def publish_velocity():
    # Initialize the ROS node
    rospy.init_node('velocity_publisher', anonymous=True)
    
    # Create a publisher object for publishing messages to the /cmd_vel topic
    vel_publisher_1 = rospy.Publisher('/robot_1/cmd_vel', Twist, queue_size=10)
    vel_publisher_2 = rospy.Publisher('/robot_2/cmd_vel', Twist, queue_size=10)
    
    # Set the loop rate (in Hz)
    rate = rospy.Rate(10) # 10Hz

    while not rospy.is_shutdown():
        # Create a new Twist message
        vel_msg = Twist()

        # Set linear velocity
        vel_msg.linear.x = 0.3 # Forward velocity
        vel_msg.linear.y = 0.0 # No sideways velocity
        vel_msg.linear.z = 0.0 # No vertical velocity

        # Set angular velocity
        vel_msg.angular.x = 0.0 # No roll
        vel_msg.angular.y = 0.0 # No pitch
        vel_msg.angular.z = 0.0 # No yaw

        # Publish the message
        vel_publisher_1.publish(vel_msg)
        vel_publisher_2.publish(vel_msg)
        
        # Log info
        rospy.loginfo("Published velocity command: linear x=%s", vel_msg.linear.x)
        
        # Sleep for the remainder of the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_velocity()
    except rospy.ROSInterruptException:
        pass
