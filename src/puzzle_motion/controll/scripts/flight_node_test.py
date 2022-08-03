#!/usr/bin/env python3
# coding=utf-8

import rospy
from takeoff_common.takeoffpy import MavController, AutoPilot
from re import X
import numpy as np

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)
def creating_position(radius, Divider):
    segment=6.28/Divider
    poses = np.empty(([Divider, 2]), dtype="float64")
    for i in range(Divider):
        array=pol2cart(radius, i*segment)
        poses[i] = array[:]
    return poses
if __name__ == "__main__":
    rospy.init_node("position_hold_test", anonymous=True)
    # Считываем параметры из launch-файла
    altitude = rospy.get_param("~altitude", 0.5)
    autopilot_type = rospy.get_param("~autopilot_type", AutoPilot.PX4)
    use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
    radius=rospy.get_param("~radius", 5)
    divider=rospy.get_param("~divider", 10)  
    poses=creating_position(radius, divider)
    # Создаем объект дрона и запускаем его в воздух
    drone = MavController.create_controller(autopilot_type, use_vision_odometry=use_vision_odometry)
    try:
        drone.takeoff(height=altitude)
        drone.follow_trajectory(poses, altitude, tolerance=0.7)
        rate = rospy.Rate(10) # 10hz
        drone.send_pos(0, 0, altitude)
        drone.land()
        
    except Exception as e:
        rospy.logerr(e)
        drone.land()
    else:
        drone.land()
