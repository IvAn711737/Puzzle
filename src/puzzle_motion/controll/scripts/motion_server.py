#!/usr/bin/env python3
# coding=utf-8

import rospy
import time

from controll.srv import Controll, ControllResponse
from takeoff_common.takeoffpy import MavController, AutoPilot

class ControllHandler:
    """Класс для обработки запросов к сервису takeoff"""

    def __init__(self, safe_zone, safe_max_height, safe_min_height, tolerance):
        ap_type = rospy.get_param("~ap_type", AutoPilot.PX4)
        use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
        self.drone = MavController.create_controller(ap_type, use_vision_odometry=use_vision_odometry)
        self.is_takeoff=False
        self.safe_zone=safe_zone                # Размер стороны квадрата в метрах за приделы которого дрон не должен вылетать ВАЖНО!нуливая точка (точка взлёта находится в центре этого квадрата)
        self.safe_max_height=safe_max_height    # Ограничение максимальной высоты подёма в метрах
        self.safe_min_height=safe_min_height    # Ограничивает минмальную допустимую высоту в метрах
        self.tolerance=tolerance                # Растояние на котором точка будет считать с достигнутой 

    def handle_takeoff(self, req):
        """Обрабатывает запрос к сервису takeoff_landing

        Args:
            req (catki): запрос к сервису, содержащий высоту и требование посадки

        Returns:
            ControllResponse: True - удачно взлет/приземлился, False - нет
        """
        try:
            if req.land:
                self.drone.land()
                self.is_takeoff=False
            elif abs(req.x)>self.safe_zone/2 or abs(req.y)>self.safe_zone/2 or req.z>self.safe_max_height or req.z<self.safe_min_height:
                rospy.logerr("Выход за приделы безопастной зоны, команда отклонена")
            elif self.is_takeoff==False:
                self.is_takeoff=True
                self.drone.takeoff(req.z)
                self.drone.go2point(x=req.x, y=req.y, z=req.z, tolerance=self.tolerance)
                
            else:
                 tic = time.perf_counter()
                 self.drone.go2point(x=req.x, y=req.y, z=req.z, tolerance=self.tolerance)
                 toc = time.perf_counter()
                 time_fly=toc - tic
                 rospy.loginfo(" Перемещение занялоe %f секунд", time_fly)
            return ControllResponse(True)
        except:
            return ControllResponse(False)
        
        
def flight_control_server():
    """Запускает сервер сервиса flight_control """
    rospy.init_node('flight_control')
    safe_zone = rospy.get_param("~safe_zone")
    safe_max_height = rospy.get_param("~safe_max_height")
    safe_min_height=rospy.get_param("~safe_min_height") 
    tolerance=rospy.get_param("~tolerance") 
    th = ControllHandler(safe_zone, safe_max_height, safe_min_height, tolerance)
    takeoff_srv = rospy.Service('flight_control', Controll, th.handle_takeoff)
    print("Ready to takeoff or land")
    rospy.spin()

if __name__ == "__main__":
    # Считываем параметры из launch-файла
    flight_control_server()
