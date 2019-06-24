#! /usr/bin/env python
import rospy
import actionlib
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
import math

class OdomAnalysis(object):
    def __init__(self, topic_name = '/sphero/imu/data3'):
        pass

    def distance_vector_cal(self, start_point, end_point):
        distVec = Vector3()
        distVec.x = start_point.x - end_point.x
        distVec.y = start_point.y - end_point.y
        distVec.z = start_point.z - end_point.z
        return distVec

    def cal_distance(self, distance_vector):
        dist = math.sqrt( (math.pow(distance_vector.x, 2)) + (math.pow(distance_vector.y, 2)) + (math.pow(distance_vector.z, 2)))
        return dist

    def get_dist(self, odom_array):
        if len(odom_array) > 1:
            start_point = odom_array[0].pose.pose.position
            end_point = odom_array[len(odom_array)-1].pose.pose.position

            distance_vector = self.distance_vector_cal(start_point, end_point)
            rospy.loginfo("Distance Vector: " + str(distance_vector))

            calcd_distance = self.cal_distance(distance_vector)
            rospy.loginfo("Distance calculated: " + str(calcd_distance))
            return calcd_distance
        else:
            rospy.logerr("Odom array should have minimum 2 elements")


def check_if_out_of_maze(goal_dist, odom_array):
  OdomAnalysisobj = OdomAnalysis()
  distance = OdomAnalysisobj.get_dist(odom_array)
  rospy.loginfo("distance moved by sphero: "+ str(distance))
  return distance > goal_dist
