#!/usr/bin/env python3
# coding=utf-8

import rospy
import time

from takeoff_common.takeoffpy import MavController, AutoPilot
from re import X
import numpy as np
R=4 # Радиус окружности по которой будет летать дрон
Divider=4 # Чтсло точек на которое будет разбита окружность 
takeof_flag=0

poses=[[0, 0], [0.5, 0.5], [-0.5, 0.5], [-0.5, -0.5], [0.5, -0.5], [0, 0]]
if __name__ == "__main__":
    rospy.init_node("position_hold_test", anonymous=True)
    # Считываем параметры из launch-файла
    altitude = rospy.get_param("~altitude", 0.5)
    autopilot_type = rospy.get_param("~autopilot_type", AutoPilot.PX4)
    use_vision_odometry = rospy.get_param("~use_vision_odometry", False)

    # Создаем объект дрона и запускаем его в воздух
    drone = MavController.create_controller(autopilot_type, use_vision_odometry=use_vision_odometry)
    drone.takeoff(height=altitude)
    for i in range(len(poses)):
        try:
            tic = time.perf_counter()
            drone.go2point(poses[i] [0], poses[i] [1], altitude, tolerance=0.1)
            toc = time.perf_counter()
            if i>0:
                rospy.loginfo(f"Перемещение из точки {poses[i-1] [0], poses[i-1] [1] } в точку {poses[i] [0], poses[i] [1]}  заняло {toc - tic:0.4f} секунд")
        except Exception as e:
            rospy.logerr(e)
    drone.land()
