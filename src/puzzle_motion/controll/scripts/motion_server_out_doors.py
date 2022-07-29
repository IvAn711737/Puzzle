#!/usr/bin/env python3
# coding=utf-8

import rospy
import time

# from controll.srv import Takeoff, Response
from controll.srv import Controll, ControllResponse
from takeoff_common.takeoffpy import MavController, AutoPilot

class TakeoffHandler:
    """Класс для обработки запросов к сервису takeoff"""

    def __init__(self):
        ap_type = rospy.get_param("~ap_type", AutoPilot.PX4)
        use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
        self.drone = MavController.create_controller(ap_type, use_vision_odometry=use_vision_odometry)
        self.is_takeoff=False
        self.safe_zone=4 # Размер стороны квадрата в метрах за приделы которого дрон не должен вылетать ВАЖНО!нуливая точка (точка взлёта находится в центре этого квадрата)
        self.safe_max_height=2.5 # Ограничение максимальной высоты подёма в метрах
        self.safe_min_height=0.2 # Ограничивает минмальную допустимую высоту в метрах

    def handle_takeoff(self, req):
        """Обрабатывает запрос к сервису takeoff_landing

        Args:
            req (catki): запрос к сервису, содержащий высоту и требование посадки

        Returns:
            ControllResponse: True - удачно взлет/приземлился, False - нет
        """
        print(req)
        try:
            if req.land:
                self.drone.land()
                self.is_takeoff=False
            elif abs(req.x)>self.safe_zone/2 or abs(req.y)>self.safe_zone/2 or req.z>self.safe_max_height or req.z<self.safe_min_height:
                print ("выход за приделы безопастной зоны")
            elif self.is_takeoff==False:
                self.is_takeoff=True
                self.drone.takeoff(req.z)
                self.drone.go2point(x=req.x, y=req.y, z=req.z, tolerance=0.3)
                
            else:
                 tic = time.perf_counter()
                 self.drone.go2point(x=req.x, y=req.y, z=req.z, tolerance=0.3)
                 toc = time.perf_counter()
                 print(f"Перемещение заняло {toc - tic:0.4f} секунд")
            return ControllResponse(True)
        except:
            return ControllResponse(False)
        
        
def flight_control_server():
    """Запускает сервер сервиса flight_control """
    rospy.init_node('flight_control')
    th = TakeoffHandler()
    takeoff_srv = rospy.Service('flight_control', Controll, th.handle_takeoff)
    print("Ready to takeoff or land")
    rospy.spin()

if __name__ == "__main__":
    flight_control_server()
