#!/usr/bin/python3

import rospy as rp
from geometry_msgs.msg import Twist
import random

from exercise2.srv import CommandTest,CommandTestResponse

def handle_reqeust(req):
    print("Got", req.type)
    movement_type = req.type
    duration = req.duration

    pub = rp.Publisher("/turtle1/cmd_vel", Twist, queue_size=100)

    if movement_type == "circle":
        twitst = Twist()
        twitst.linear.x = 1
        twitst.angular.z = 1
    elif movement_type == "rectangle":
        twitst = Twist()
        twitst.linear.x = 1
        twitst.angular.z = 0
    elif movement_type == "triangle":
        twitst = Twist()
        twitst.linear.x = 1
        twitst.angular.z = 0.5
    elif movement_type == "random":
        twitst = Twist()
        twitst.linear.x = random.randint(0, 10)
        twitst.angular.z = random.randint(0, 10)
    else:
        rp.logerr("Unknown movement type: %s", movement_type)
        return CommandTestResponse("Unknown movement type: %s" % movement_type)
    

    start_time = rp.Time.now()
    while rp.Time.now() - start_time < rp.Duration(duration):
        pub.publish(twitst)
        rp.sleep(0.1)

    stop_twist = Twist()
    pub.publish(stop_twist)

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
