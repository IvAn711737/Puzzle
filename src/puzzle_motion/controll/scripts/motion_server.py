#!/usr/bin/env python
# coding=utf-8

import rospy

# from controll.srv import Takeoff, Response
from controll.srv import Takeoff1, Takeoff1Response
from takeoff_common.takeoffpy import MavController, AutoPilot

safe_zone=10 # Размер стороны квадрата за приделы которого дрон не должен вылетать ВАЖНО!нуливая точка (точка взлёта находится в центре этого квадрата)

class TakeoffHandler:
    """Класс для обработки запросов к сервису takeoff"""

    def __init__(self):
        ap_type = rospy.get_param("~ap_type", AutoPilot.PX4)
        use_vision_odometry = rospy.get_param("~use_vision_odometry", False)
        self.drone = MavController.create_controller(ap_type, use_vision_odometry=use_vision_odometry)

    def handle_takeoff(self, req):
        """Обрабатывает запрос к сервису takeoff_landing

        Args:
            req (Takeoff): запрос к сервису, содержащий высоту и требование посадки

        Returns:
            TakeoffResponse: True - удачно взлет/приземлился, False - нет
        """
        print(req)
        try:
            if abs(req.x)>safe_zone/2 or abs(req.y)>safe_zone/2:
                print ("выход за приделы безопастной зоны")
                self.drone.hover_timer_cb()
            elif req.land:
                self.drone.land()
                # self.drone.land()
            else:
                self.drone.takeoff(req.z)
                self.drone.go2point(x=req.x, y=req.y, z=req.z, tolerance=0.1)
            return Takeoff1Response(True)
        except:
            return Takeoff1Response(False)
        
        
def takeoff_landing_server():
    """Запускает сервер сервиса takeoff_landing """
    rospy.init_node('takeoff_server')
    th = TakeoffHandler()
    takeoff_srv = rospy.Service('takeoff_landing', Takeoff1, th.handle_takeoff)
    print("Ready to takeoff or land")
    rospy.spin()


if __name__ == "__main__":
    takeoff_landing_server()
