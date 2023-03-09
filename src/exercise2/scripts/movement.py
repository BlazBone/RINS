#!/usr/bin/python3

import rospy as rp
from geometry_msgs.msg import Twist
import random

def rectangle_movement(step):
  rp.loginfo("Rectangle movement")
  twist = Twist()
  twist.linear.x = 1
  step = step % 20

  if step % 5 == 0:
    twist.linear.x = 0
    twist.angular.z = 1.57 #(90 / 360) * 2 * 3.14

  return twist


def triangle_movement(step):
  rp.loginfo("Triangle movement")
  twist = Twist()
  twist.linear.x = 1
  step = step % 9

  if step % 3 == 0:
    twist.linear.x = 0
    twist.angular.z = (120 / 360) * 2 * 3.14

  return twist


def circle_movement(step):
  rp.loginfo("Circle movement")
  twist = Twist()
  twist.linear.x = 1
  twist.angular.z = (70 / 360) * 2 * 3.14

  return twist


def random_movement(step):
  rp.loginfo("Random movement")
  twist = Twist()
  twist.linear.x = rp.get_param("~scale_linear") * random.random()
  twist.angular.z = rp.get_param('~scale_angular') * 2 * (random.random() - 0.5) # +-

  return twist

def move(movement_type: str, duration: int ) -> str:
  rp.init_node('movement')
  pub = rp.Publisher('cmd_vel', Twist, queue_size = 1000)

  if movement_type == "rectangle":
      function = rectangle_movement
  elif movement_type == "triangle":
      function = triangle_movement
  elif movement_type == "circle":
      function = circle_movement
  elif movement_type == "random":
      function = random_movement
  else:
      rp.logerr("Unknown movement type: %s", movement_type)
      return movement_type

  rp.loginfo("Starting movement: %s", movement_type)
  start_time = rp.Time.now()
  # write a loop that runs for 'duration' seconds
  rp.loginfo("Duration: %s", duration)
  
  r = rp.Rate(1)
  while rp.Time.now() < start_time + rp.Duration(duration):
      twist = function(step)
      pub.publish(twist)
      step = step + 1.0
      r.sleep()

  stop_twist = Twist()
  pub.publish(stop_twist)
  rp.loginfo("Movement finished: %s", movement_type)
  return movement_type

def movement():

  # For the turtle simulation map the topic to /turtle1/cmd_vel
  # For the turtlebot simulation and Turtlebot map the topic to /cmd_vel_mux/input/navi
  rp.init_node('movement')
  pub = rp.Publisher('cmd_vel', Twist, queue_size = 1000)
  r = rp.Rate(2)


  while not rp.is_shutdown():
    pub.publish(Twist())
    r.sleep()

if __name__ == '__main__':
    movement()