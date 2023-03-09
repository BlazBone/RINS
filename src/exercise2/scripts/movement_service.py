#!/usr/bin/python3

import rospy as rp
from geometry_msgs.msg import Twist
import random
import time
from exercise2.srv import CommandTest,CommandTestResponse

def rectangle_movement(step):

  twist = Twist()
  twist.linear.x = 0.5
  step = step % 20

  if step % 5 == 0:
    twist.linear.x = 0
    twist.angular.z = 1.57 #(90 / 360) * 2 * 3.14

  return twist


def triangle_movement(step):

  twist = Twist()
  twist.linear.x = 1
  step = step % 9

  if step % 3 == 0:
    twist.linear.x = 0
    twist.angular.z = (120 / 360) * 2 * 3.14

  return twist


def circle_movement(step):

  twist = Twist()
  twist.linear.x = 1
  twist.angular.z = (70 / 360) * 2 * 3.14

  return twist


def random_movement(step):

  twist = Twist()
  twist.linear.x = rp.get_param("~scale_linear") * random.random()
  twist.angular.z = rp.get_param('~scale_angular') * 2 * (random.random() - 0.5) # +-

  return twist


def handle_reqeust(req):
    print("Got", req.type)
    movement_type = req.type
    duration = req.duration

    pub = rp.Publisher("cmd_vel", Twist, queue_size=100)

    time_end = time.time() + duration
    step = 0

    

    if movement_type == "circle":
        function = circle_movement
    elif movement_type == "rectangle":
        function = rectangle_movement
    elif movement_type == "triangle":
        function = triangle_movement
    elif movement_type == "random":
        function = random_movement
    else:
        rp.logerr("Unknown movement type: %s", movement_type)
        return CommandTestResponse("Unknown movement type: %s" % movement_type)
    
    rp.loginfo("Starting movement: %s", movement_type)
    start_time = rp.Time.now()
    # write a loop that runs for 'duration' seconds
    while rp.Time.now() < start_time + rp.Duration(duration):
        twist = function(step)
        pub.publish(twist)
        step = step + 1.0
        rp.sleep(0.1)

    stop_twist = Twist()
    pub.publish(stop_twist)

    response = CommandTestResponse()
    response.type = movement_type

    return response


def movement_server():
    rp.rate(2)
    rp.init_node("movement_server")
    s = rp.Service("movement_service", CommandTest, handler=handle_reqeust)
    print("Ready to move the turtle")
    rp.spin()

if __name__ == "__main__":
    movement_server()
