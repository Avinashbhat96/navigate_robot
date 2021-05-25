#!/usr/bin/env python

import rospy
from actionlib import SimpleActionServer, SimpleActionClient
from navigate_robot.msg import NavigateRobotAction, NavigateRobotGoal, NavigateRobotResult, NavigateRobotFeedback
from move_base_msgs.msg import MoveBaseAction, MoveBaseFeedback, MoveBaseGoal, MoveBaseResult
from datetime import date, datetime

class NavigateRobotServer:

    X = [0.5, 1, 0.5, -0.5, -1]
    Y = [-1, -0.5, -1, 0.5, 1]
    WQ = [1, 1, 1, 1, 1]
    MAP = rospy.get_param("/nav_robot/map_name")

    def __init__(self):
        rospy.init_node("nav_robot")
        self._start_pos = MoveBaseGoal()
        self._start_pos_assigned = False
        self._server_name = "navigate_robot"
        self._feedback = NavigateRobotFeedback()
        self._result = NavigateRobotResult()
        self._movebase_server = SimpleActionClient("/move_base", MoveBaseAction)
        self._movebase_server.wait_for_server()
        self._server = SimpleActionServer(self._server_name, NavigateRobotAction , self._callback, auto_start=False)
        self._server.start()
        rospy.loginfo("Navigation server has been started, you can call the action now!")
        

    def _callback(self, goal):
        start_time = datetime.now()
        goals_to_reach = goal.num_goals
        success = True
        goal_reached = 0
        self._result.state = 1
        for i in range(goals_to_reach):
            if i  >= len(self.X):
                rospy.loginfo("All goals available are successfully tried")
                break
            
            if self._server.is_preempt_requested():
                #self._movebase_server.cancel_goal()
                rospy.loginfo("Server has been preempted")
                self._server.set_preempted()
                self._result.state = 2
                success = False
                break

            else:
                goal_point = self.set_goal(i)
                self._movebase_server.send_goal(goal_point, feedback_cb=self.movebase_feedback)
                self._feedback.current_state = "Goal assigned"
                rospy.sleep(2)
                self._movebase_server.wait_for_result()
                movebase_state = self._movebase_server.get_state()
                print(movebase_state)
                if movebase_state == 3:
                    self._feedback.current_state = "Goal reached"
                    rospy.loginfo("Goal reached")
                    goal_reached += 1
                elif movebase_state == 4:
                    rospy.loginfo("Could not reach the goal")
                    self._feedback.current_state = "Goal missed"
                self._server.publish_feedback(self._feedback)
                self._result.state = 3
        
        ret = self.nav_start_pos() 

        
        rospy.loginfo("{server}: successfully reached {goal} goals".format(server=self._server_name, goal=goal_reached))
        end_time = datetime.now()
        rospy.loginfo("Total time taken from this action to navigate to points is: {}".format(end_time - start_time))
        if success:
            self._server.set_succeeded(self._result)

    def movebase_feedback(self, fb):
        """
        Current position will be obtained here, if required can be accessed.
        """
        if not self._start_pos_assigned:
            self._start_pos.target_pose.header.frame_id = self.MAP
            self._start_pos.target_pose.header.stamp = rospy.Time.now()
            self._start_pos.target_pose.pose.position.x = fb.base_position.pose.position.x
            self._start_pos.target_pose.pose.position.y = fb.base_position.pose.position.y
            self._start_pos.target_pose.pose.orientation.w = fb.base_position.pose.orientation.w
            self._start_pos_assigned = True
        else:
            pass

    def set_goal(self, i):
        goal_point = MoveBaseGoal()
        goal_point.target_pose.header.frame_id = self.MAP
        goal_point.target_pose.header.stamp = rospy.Time.now()
        goal_point.target_pose.pose.position.x = self.X[i]
        goal_point.target_pose.pose.position.y = self.Y[i]
        goal_point.target_pose.pose.orientation.w = self.WQ[i]
        rospy.loginfo("Current goal: x={}, Y={}".format(self.X[i], self.Y[i]))

        return goal_point
    
    def nav_start_pos(self):
        rospy.loginfo("Moving to starting position")
        self._movebase_server.send_goal(self._start_pos, feedback_cb=self.movebase_feedback)
        rospy.sleep(2)
        self._movebase_server.wait_for_result()
        movebase_state = self._movebase_server.get_state()

        if movebase_state == 3:
            rospy.loginfo("Reached starting positoin")
            return True
        else:
            rospy.loginfo("Could not reach the starting position")
            return False


if __name__ == '__main__':
    nav_server = NavigateRobotServer()
    rospy.spin()