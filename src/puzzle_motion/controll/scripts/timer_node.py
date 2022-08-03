#!/usr/bin/env python3
# coding=utf-8

import rospy
import time
from takeoff_common.takeoffpy import MavController, AutoPilot

def listener():
    """Запускает сервер сервиса takeoff_landing """
    rospy.init_node()
    rospy.Subscriber ("/mavros/setpoint_position/local")
    th = TakeoffHandler()
    takeoff_srv = rospy.Service('takeoff_landing', Controll, th.handle_takeoff)
    print("Ready to takeoff or land")
    rospy.spin()

if __name__ == "__main__":
    listener()