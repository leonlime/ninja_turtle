#!/usr/bin/env python

# libs:
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

# This class contain all the aspects of robot, like a nodes and metods
class ninja_turtle:
  def __init__(self):
    # Creates a node 
    rospy.init_node('ninjaturtle_controller', anonymous=True)
    # Publisher for the topic '/turtle1/cmd_vel'
    self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    # Subscriber for the topic '/turtle1/pose'
    self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

    self.pose = Pose()
    self.rate = rospy.Rate(10)
  
  






