#!/usr/bin/env python3
import rospy, actionlib, time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

GOALS = [
    ("goal1_start",  0.030306, -0.033085, -0.00600194, 0.999982),
    ("goal3_right",  2.732700, -1.939300,  0.00652463, 0.999972),
    ("goal4_back",   2.640200, -1.969990, -0.70577400, 0.708434),
]

class Nav:
    def __init__(self):
        rospy.init_node("goals_launcher")
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("waiting for move_base ...")
        self.client.wait_for_server()

    def go(self, x, y, z, w, name, retry=3):
        for attempt in range(retry):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.z = z
            goal.target_pose.pose.orientation.w = w
            self.client.send_goal(goal)
            rospy.loginfo(f"[{name}] try {attempt+1}/{retry} -> ({x:.3f},{y:.3f})")
            if self.client.wait_for_result(rospy.Duration(90)):
                if self.client.get_state() == GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"[{name}] OK")
                    return True
            self.client.cancel_goal()
            time.sleep(1)
        rospy.logwarn(f"[{name}] FAIL after {retry} tries")
        return False

if __name__ == "__main__":
    nav = Nav()
    for name, x, y, z, w in GOALS:
        nav.go(x, y, z, w, name)
        time.sleep(1)
    rospy.loginfo("all goals done")
