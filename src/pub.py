#!/usr/bin/env python

# libs:
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

# This class contain all the aspects of robot, like a nodes and metods
class NinjaTurtle:
  def __init__(self):
    # Print test
    print('Ninja turtle class is working')
    # Creates a node 
    rospy.init_node('ninjaturtle_controller', anonymous=True)
    # Publisher for the topic '/turtle1/cmd_vel'
    self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    # Subscriber for the topic '/turtle1/pose'
    self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.updatePose)
    # Pose var using a Pose function from turtlesim.msg
    self.pose = Pose()
    # Setup the loop rate to 10hz
    self.rate = rospy.Rate(10)
  
  # Callback function. called when a new msg is received by the subscriber
  def updatePose(self, data):
    self.pose = data
    self.pose.x = round(self.pose.x, 4)
    self.pose.y = round(self.pose.y, 4)

  # Euclidean distance between current pose and the goal pose
  def euclideanDistance(self, goal_pose):
    return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
  
  # Set the linear vel
  def linearVel(self, goal_pose, constant=1.5):
    return constant * self.euclideanDistance(goal_pose)

  # Set the angle
  def steeringAngle(self, goal_pose):
    return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

  # Set the angular vel
  def angularVel(self, goal_pose, constant=6):
    return constant * (self.steeringAngle(goal_pose) - self.pose.theta)

  # Move the turtle to the goal point
  def move2Goal(self):
    # move2goal print test
    print('Move to goal function is working')

    while True:
      goal_pose = Pose()

      # Get the input from the user.
      goal_pose.x = input("X position: ")
      goal_pose.y = input("Y position: ")
      print('----------')

      # Please, insert a number slightly greater than 0 (e.g. 0.01).
      distance_error = 0.1
      vel_msg = Twist()

      # Main loop move2goal function
      while self.euclideanDistance(goal_pose) >= distance_error:
        vel_msg.linear.x = self.linearVel(goal_pose)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self.angularVel(goal_pose)

        # Publishing our vel_msg
        self.velocity_publisher.publish(vel_msg)

        # Publish at the desired rate.
        self.rate.sleep()

      # Stopping our robot after the movement is over.
      vel_msg.linear.x = 0
      vel_msg.angular.z = 0
      self.velocity_publisher.publish(vel_msg)

    # If we press control + C, the node will stop.
    rospy.spin()

# Main function
if __name__ == "__main__":
  try:
    leonardo = NinjaTurtle()
    leonardo.move2Goal()
  except rospy.ROSInterruptException:
    pass  



