#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

GOALS = [
        (-0.001434326171875, 0.29035043716430664, -0.001434326171875),
        (-0.001434326171875, 0.29035043716430664, -0.001434326171875),
        (-0.001434326171875, 0.29035043716430664, -0.001434326171875),
        (-0.8019881844520569, -0.4780222773551941, -0.001434326171875),
        (-0.10407786071300507, -1.9015296697616577, -0.005340576171875),
        (0.33748140931129456, -1.3737103939056396, -0.005340576171875),
        (1.6469016075134277, -1.9848524332046509, -0.005340576171875)
        ]

GOALS = [
        (-0.7952509522438049, 1.5871058702468872, -0.001434326171875),
        (-0.7952509522438049, 1.5871058702468872, -0.001434326171875),
        (-0.7952509522438049, 1.5871058702468872, -0.001434326171875),
        (-0.5639151334762573, 0.8585555553436279, -0.001434326171875),
        (-0.5457677245140076, 0.17789430916309357, -0.001434326171875),
        (-0.24812187254428864, -0.8512775301933289, -0.001434326171875),
        (0.6777873635292053, -1.8661431074142456, -0.001434326171875)
        ]

NEXT_GOAL = True

STATUS_DICT = {
    0:"The goal has yet to be processed by the action server",
    1:"The goal is currently being processed by the action server",
    2:"The goal received a cancel request after it started executing and has since completed its execution (Terminal State)",
    3:"The goal was achieved successfully by the action server (Terminal State)",
    4:"The goal was aborted during execution by the action server due to some failure (Terminal State)",
    5:"The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)",
    6:"The goal received a cancel request after it started executing and has not yet completed execution",
    7:"The goal received a cancel request before it started executing but the action server has not yet confirmed that the goal is canceled",
    8:"The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)",
    9:"An action client can determine that a goal is LOST. This should not be sent over the wire by an action server",
}
def callback(data: GoalStatusArray):
    global NEXT_GOAL
    slist = data.status_list
    if len(slist) == 0:
        NEXT_GOAL = True
        return
    # rospy.loginfo(f"status: {slist[-1].status}: The goal is currently")
    rospy.loginfo(f"status {slist[-1].status}: {STATUS_DICT[slist[-1].status]}")
    if slist[-1].status == 3:
        NEXT_GOAL = True
    elif slist[-1].status not in (0, 1,):
        NEXT_GOAL = True

def move_to_point(id: int, goals_idx: int) -> PoseStamped: 
    msg = PoseStamped()
    msg.header.seq = id
    msg.header.frame_id = "map"
    msg.pose.orientation.w = 1
    msg.pose.position.x = GOALS[goals_idx][0]
    msg.pose.position.y = GOALS[goals_idx][1]
    # msg.header.stamp = rospy.get_time()
    return msg
        
def main():
    global NEXT_GOAL
    rospy.init_node("goal_pub_node", anonymous=True)
    TOPIC="/move_base_simple/goal"
    pub = rospy.Publisher(TOPIC, PoseStamped, queue_size=10)
    rospy.loginfo(f"Publishing on {TOPIC}")
    sub = rospy.Subscriber("/move_base/status", GoalStatusArray, callback=callback)

    rate = rospy.Rate(1)
    
    seq_id = 0
    goals_idx = 0
    while not rospy.is_shutdown():
        if goals_idx >= len(GOALS) and NEXT_GOAL == True:
            rospy.loginfo("All goals reached, shutting down.")
            break

        if NEXT_GOAL:
            msg = move_to_point(seq_id, goals_idx) 
            seq_id += 1
            goals_idx += 1
            rospy.loginfo(f"Publishing goal {seq_id}.")
            pub.publish(msg)
            rospy.loginfo(f"Goal {seq_id} published")
            rospy.loginfo(msg)
            NEXT_GOAL = False
        else:
            # rospy.spin()
            pass

        rate.sleep()
        

if __name__ == "__main__":
    main()
