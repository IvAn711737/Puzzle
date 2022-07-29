#!/usr/bin/env python3
# coding=utf-8

import rospy

from controll.srv import Controll, ControllResponse
from takeoff_common.takeoffpy import MavController, AutoPilot


class TakeoffHandler:
    """Класс для обработки запросов к сервису takeoff"""

    def __init__(self):
        ap_type = rospy.get_param("~ap_type", AutoPilot.PX4)
        use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
        self.drone = MavController.create_controller(ap_type, use_vision_odometry=use_vision_odometry)
        self.is_takeoff=False
        self.safe_zone=3 # Размер стороны квадрата в метрах за приделы которого дрон не должен вылетать ВАЖНО!нуливая точка (точка взлёта находится в центре этого квадрата)
        self.safe_max_height=1.5 # Ограничение максимальной высоты подёма в метрах
        self.safe_min_height=0.2 # Ограничивает минмальную допустимую высоту в метрах
    def handle_takeoff(self, req):
        """Обрабатывает запрос к сервису takeoff_landing

        Args:
            req (Takeoff): запрос к сервису, содержащий высоту и требование посадки

        Returns:
            TakeoffResponse: True - удачно взлет/приземлился, False - нет
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
                self.drone.go2point(x=req.x, y=req.y, z=req.z, tolerance=0.1)
                
            else:
                 self.drone.go2point(x=req.x, y=req.y, z=req.z, tolerance=0.1)
            return ControllResponse(True)
        except:
            return ControllResponse(False)
        
        
def takeoff_landing_server():
    """Запускает сервер сервиса takeoff_landing """
    rospy.init_node('takeoff_server')
    th = TakeoffHandler()
    takeoff_srv = rospy.Service('takeoff_landing', Controll, th.handle_takeoff)
    print("Ready to takeoff or land")
    rospy.spin()


if __name__ == "__main__":
    takeoff_landing_server()
