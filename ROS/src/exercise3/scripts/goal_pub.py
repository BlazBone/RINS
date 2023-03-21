#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

GOALS = [
        (-0.001434326171875, 0.29035043716430664, -0.001434326171875),
        (-0.8019881844520569, -0.4780222773551941, -0.001434326171875),
        (-0.10407786071300507, -1.9015296697616577, -0.005340576171875),
        (0.33748140931129456, -1.3737103939056396, -0.005340576171875),
        (1.6469016075134277, -1.9848524332046509, -0.005340576171875)
        ]

def callback(data):
    print(data) 

def move_to_point(id: int) -> PoseStamped: 
    msg = PoseStamped()
    msg.frame_id = "map"
    msg.pose.orientation.w = 1
    msg.pose.position.x = GOALS[id][0]
    msg.pose.position.y = GOALS[id][1]
    msg.header.stamp = rospy.Time.now()
    return msg

def main():
    NEXT_GOAL = True
    rospy.init_node("goal_pub_node")

    pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    sub = rospy.Subscriber("/move_base/status", GoalStatusArray, callback=callback)

    rate = rospy.Rate(1)
    
    seq_id = 0
    while not rospy.is_shutdown():
        if NEXT_GOAL:
            msg = move_to_point(seq_id) 
            seq_id += 1
            rospy.loginfo(f"Publishing goal {seq_id}.")
            pub.publish(msg)
            NEXT_GOAL = False
        else:
            rospy.spin()


        if seq_id >= 5:
            rospy.loginfo("All goals reached, shutting down.")
            break
        rate.sleep()
        

if __name__ == "__main__":
    main()
