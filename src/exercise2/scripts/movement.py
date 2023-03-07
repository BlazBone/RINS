#!/usr/bin/python3

import rospy as rp
from geometry_msgs.msg import Twist

def rectangle_movement(step):

  twist = Twist()
  twist.linear.x = 0.1
  step = step % 80

  if step % 20 == 0:
    twist.linear.x = 0
    twist.angular.z = 1.57 #(90 / 360) * 2 * 3.14

  return twist

def movement():

  pub = rp.Publisher('cmd_vel', Twist, queue_size = 1000)
  # For the turtle simulation map the topic to /turtle1/cmd_vel
  # For the turtlebot simulation and Turtlebot map the topic to /cmd_vel_mux/input/navi
  rp.init_node('movement')

  r = rp.Rate(10000)

  step = 0.0
  
  while not rp.is_shutdown():
    twist = rectangle_movement(step)
    pub.publish(twist)
    step = step + 2
    r.sleep()

if __name__ == '__main__':
    movement()
