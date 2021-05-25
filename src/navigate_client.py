#!/usr/bin/env python

import rospy
from actionlib import SimpleActionClient
from navigate_robot.msg import NavigateRobotAction, NavigateRobotGoal, NavigateRobotFeedback, NavigateRobotResult

class NavigateRobotClient:

    def __init__(self):
        rospy.init_node("navigate_robot_client")
        self._client_name = "navigate_robot"
        self._client = SimpleActionClient(self._client_name, NavigateRobotAction)
        self._client.wait_for_server()
        self._goal = NavigateRobotGoal()
    
    def send_goals(self):
        self._goal.num_goals = int(rospy.get_param("/navigate_robot_client/goals_count"))
        self._client.send_goal(self._goal)
        rospy.sleep(2)
        self._client.wait_for_result()
        if self._client.get_state() == 3:
            rospy.loginfo("Reached all possible points")
        elif self._client.get_state() == 2:
            rospy.loginfo("Could not be reached goal points, Preempted!")
    

if __name__ == '__main__':
    client = NavigateRobotClient()
    client.send_goals()