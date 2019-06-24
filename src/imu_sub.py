#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

class ImuTopicReader(object):
    def __init__(self, topic_name = '/sphero/imu/data3'):
        self._topic_name = topic_name
        self._sub = rospy.Subscriber(self._topic_name, Imu, self.topic_callback)
        self._imu_data = Imu()

    def topic_callback(self, msg_data):
        self._imu_data = msg_data
        rospy.logdebug(self._imu_data)

    def get_imu_data(self):
        return self._imu_data

    def collision_detection(self):
        a_x = self._imu_data.linear_acceleration.x
        a_y = self._imu_data.linear_acceleration.y
        a_z = self._imu_data.linear_acceleration.z
        list_acc = [a_x, a_y, a_z]

        _coll_acc_threshold = 7.0

        max_acc_index = list_acc.index(max(list_acc))

        _coll_direction = ""

        if abs(list_acc[max_acc_index]) > _coll_acc_threshold:
            if max_acc_index == 0:
                rospy.loginfo("Collision detected in x-axis")
                if list_acc[max_acc_index] > 0:
                    rospy.loginfo("Collision in right direction")
                    _coll_direction = "right"
                else:
                    rospy.loginfo("Collision in left direction")
                    _coll_direction = "left"

            elif max_acc_index == 1:
                rospy.loginfo("Collision detected in y-axis")
                if list_acc[max_acc_index] > 0:
                    rospy.loginfo("Collision in forward direction")
                    _coll_direction = "forward"
                else:
                    rospy.loginfo("Collision in backward direction")
                    _coll_direction = "backward"

            elif max_acc_index == 2:
                rospy.loginfo("Collision detected in z-axis")
                if list_acc[max_acc_index] > 0:
                    rospy.loginfo("Collision in up direction")
                    _coll_direction = "up"
                else:
                    rospy.loginfo("Collision in down direction")
                    _coll_direction = "down"

        _coll_dir_dict = {
            "up": _coll_direction == "up",
            "down": _coll_direction == "down",
            "left": _coll_direction == "left",
            "right": _coll_direction == "right",
            "forward": _coll_direction == "forward",
            "backward": _coll_direction == "backward"
        }

        return _coll_dir_dict





if __name__ == "__main__":
    rospy.init_node('imu_topic_publisher', log_level=rospy.INFO)
    imu_reader_object = ImuTopicReader()
    rospy.loginfo(imu_reader_object.get_imu_data())
    rate = rospy.Rate(0.5)

    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        print "shutdown time!"
        ctrl_c = True

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        data = imu_reader_object.get_imu_data()
#        print imu_reader_object.collision_detection()
        rospy.loginfo(data)
        rate.sleep()