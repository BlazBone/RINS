#!/usr/bin/python3

import rospy as rp
from geometry_msgs.msg import Twist
import random
from exercise2.srv import CommandTest,CommandTestResponse

pub = rp.Publisher('cmd_vel', Twist, queue_size = 1000)



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



def handle_reqeust(req):
    print("Got", req.type)
    movement_type = req.type
    duration = req.duration

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

    rp.loginfo("Starting movement: %s with duration %s", movement_type,duration)
    
    # get while for duration seconds
    start_time = rp.Time.now()
    r = rp.Rate(1)
    step = 0
    while rp.Time.now() < start_time + rp.Duration(duration):
      twist = function(step)
      pub.publish(twist)
      step = step + 1.0
      r.sleep()
  
    response = CommandTestResponse()
    response.type = movement_type

    return response


def movement_server():
    rp.init_node("movement_server")
    s = rp.Service("movement_service", CommandTest, handler=handle_reqeust)
    print("Ready to move the turtle")
    rp.spin()

if __name__ == "__main__":
    movement_server()
