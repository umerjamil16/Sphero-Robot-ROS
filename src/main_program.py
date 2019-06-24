#! /usr/bin/env python

import rospy
import time
import actionlib
from my_sphero_actions.msg import record_odomGoal, record_odomFeedback, record_odomResult, record_odomAction, record_odomFeedback
from nav_msgs.msg import Odometry

from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest # you import the service message python classes generated from Empty.srv.
from imu_sub import ImuTopicReader
from odom_analysis import check_if_out_of_maze
from cmd_topic_pub import CmdVelPub

#Given
#Service: Tells if the crash has happened and in which direction
#Action: Tells if time has passed or the robot has existed the maze

class MainProgram(object):
    def __init__(self):
        CmdVelPubObj = CmdVelPub()
        self.init_crash_dir_service_client()
        self.init_action_server_client()
        self.goal_dist = 2.0

        self.CmdVelPubObj = CmdVelPub()

    def init_crash_dir_service_client(self, service_name = "/crash_service_server"):
        rospy.loginfo("Waiting for service: " + service_name)
        rospy.wait_for_service(service_name) # wait for the service client /gazebo/delete_model to be running
        rospy.loginfo("Service: "+ service_name + "is available now...")
        self.direction_service = rospy.ServiceProxy(service_name, Trigger) # create the connection to the service
        self.request_object = TriggerRequest()
        #result.success, result direction

    def make_direction_request(self):
        result = self.direction_service(self.request_object) # send through the connection the request
        return result.message

    def init_action_server_client(self, action_name = "/rec_odom_action_server"):
        self.action_client = actionlib.SimpleActionClient(action_name, record_odomAction)
        rospy.loginfo('Waiting for action Server')
        self.action_client.wait_for_server()
        rospy.loginfo('Action Server Found...')

    def send_goal_to_action_server(self):
        goal = record_odomGoal()
        self.action_client.send_goal(goal, feedback_cb=self.action_feedback_callback)
        #next is state var, action completion etc

    def get_result_rec_odom(self):
        return self.action_client.get_result()

    def action_done(self):
        state_result = self.action_client.get_state()
        return (state_result >= 2)


    def action_feedback_callback(feedback):
        rospy.loginfo("Rec Odom Feedback feedback ==>"+str(feedback))

    def move_sphero(self, direction):
        self.CmdVelPubObj.move_robot(direction)

    def got_out_of_maze(self, odom_array):
        return check_if_out_of_maze(self.goal_dist, odom_array)


rospy.init_node("sphero_main_node", log_level= rospy.INFO)
MainProgramObj = MainProgram()
rate = rospy.Rate(10)
print "hello1"
MainProgramObj.send_goal_to_action_server()
print "hello2"
while not MainProgramObj.action_done():
    to_go_dir = MainProgramObj.make_direction_request()
    rospy.loginfo("Moving in dir:" + str(to_go_dir))
    MainProgramObj.move_sphero(to_go_dir)
    rate.sleep()

odom_array = MainProgramObj.get_result_rec_odom()
odom_result_arr = odom_array.result_odom_array

if MainProgramObj.got_out_of_maze(odom_result_arr):
    rospy.logingo("Out of the maze")
else:
    rospy.loginfo("In maze")

rospy.loginfo("Sphero test is finished")



