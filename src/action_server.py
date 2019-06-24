#! /usr/bin/env python
import rospy
import actionlib
from my_sphero_actions.msg import record_odomFeedback, record_odomResult, record_odomAction
from odom_analysis import check_if_out_of_maze
from odom_sub import OdomTopicReader

class OdomActionServer(object):

  # create messages that are used to publish feedback/result
    _result   = record_odomResult()

    def __init__(self, goal_dist):
        self._goal_distance = goal_dist
        self._time_avail = 120
        self.OdomTopicReaderObj = OdomTopicReader()

        # creates the action server
        self._as = actionlib.SimpleActionServer("/rec_odom_action_server", record_odomAction, self.goal_callback, False)
        self._as.start()

    def reached_dist_goal(self):
        return check_if_out_of_maze(self._goal_distance, self._result.result_odom_array)

    def goal_callback(self, goal):
        success = True
        rate = rospy.Rate(1)
    # this callback is called when the action server is called.
        for i in range(self._time_avail):
            rospy.loginfo("recording odom data - " + str(i))
            if self._as.is_preempt_requested():
                rospy.logdebug('The goal has been cancelled/preempted')
                # the following line sets the client in a preempted state (goal cancelled)
                self._as.set_preempted()
                success = False
                # we end the action loop
                break
            else:
                if not check_if_out_of_maze(self._goal_distance, self._result.result_odom_array):
                    rospy.loginfo("reading odom data and appending")
                    self._result.result_odom_array.append(self.OdomTopicReaderObj.get_odomdata())
                else:
                    rospy.loginfo("reached the goal distance")
                    break
            rate.sleep()

        if success:
            self._as.set_succeeded(self._result)
            self._result = record_odomResult # clean the var for next possible call




if __name__ == '__main__':
  rospy.init_node('odom_action_server_node')
  OdomActionServer(goal_dist = 2.0)
  rospy.spin()