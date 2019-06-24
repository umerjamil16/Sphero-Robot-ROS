#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse # you import the service message python classes generated from Empty.srv.
from imu_sub import ImuTopicReader

class CrashSrv_Server(object):
    def __init__(self, srv_name = "/crash_service_server"):
        self.ImuTopicReaderObj = ImuTopicReader()
        self._my_service = rospy.Service(srv_name, Trigger , self.my_callback) # create the Service called my_service with the defined callback
        self._response_srv = TriggerResponse()

    def my_callback(self, request):
        _coll_dict = self.ImuTopicReaderObj.collision_detection()

        collision_dir = ""
        if _coll_dict['up']:
            collision_dir = "down"
        elif _coll_dict['down']:
            collision_dir = "up"
        elif _coll_dict['left']:
            collision_dir = "right"
        elif _coll_dict['forward']:
            collision_dir = "backward"
        elif _coll_dict['backward']:
            collision_dir = "forward"

        if collision_dir != "": #there is a collision
            self._response_srv.success = True
            self._response_srv.message = collision_dir
        else:
            self._response_srv.success = False
            self._response_srv.message = "stop"#check again

        return self._response_srv

if __name__ == "__main__":
    rospy.init_node('crash_dir_service_server')
    CrashSrv_ServerObj = CrashSrv_Server()
    rospy.spin() # maintain the service open.