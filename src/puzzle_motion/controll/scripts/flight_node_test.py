#!/usr/bin/env python3
# coding=utf-8

import rospy
from takeoff_common.takeoffpy import MavController, AutoPilot
from re import X
import numpy as np
R=5 # Радиус окружности по которой будет летать дрон
Divider=100 # Чтсло точек на которое будет разбита окружность 

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)
def creating_position(R, Divider):
    segment=6.28/Divider
    poses = np.empty(([Divider, 2]), dtype="float64")
    for i in range(Divider):
        array=pol2cart(R, i*segment)
        poses[i] = array[:]
    return poses
poses=creating_position(R, Divider)
if __name__ == "__main__":
    rospy.init_node("position_hold_test", anonymous=True)
    # Считываем параметры из launch-файла
    altitude = rospy.get_param("~altitude", 0.5)
    autopilot_type = rospy.get_param("~autopilot_type", AutoPilot.PX4)
    use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
    # hover_time = int(rospy.get_param("~hover_time", 25))
    # if hover_time <= 0: hover_time = 5
    # Создаем объект дрона и запускаем его в воздух
    drone = MavController.create_controller(autopilot_type, use_vision_odometry=use_vision_odometry)
    try:
        drone.takeoff(height=altitude)
        drone.follow_trajectory(poses, altitude, tolerance=0.7)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            drone.send_pos(0, 0, 5, 6)

            rate.sleep()
        drone.land()
        # drone.init()
        

    except Exception as e:
        rospy.logerr(e)
