#if !defined(_PUZLLE_ROS_RVIZ_PAINTER_H_)
#define _PUZLLE_ROS_RVIZ_PAINTER_H_

#include "RvizPainterObject.hpp"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/Marker.h>

using geometry_msgs::Point;
using geometry_msgs::Pose;
using std_msgs::ColorRGBA;
using visualization_msgs::Marker;

class RvizPainter {
private:
  const char *_pubName;
  ros::NodeHandle &_nh;
  ros::Publisher _markersPublisher;

public:
  explicit RvizPainter(ros::NodeHandle &nh, const char *pubName = "rviz_painter") : _nh(nh), _pubName(pubName) {
    _markersPublisher = _nh.advertise<Marker>(_pubName, 100);
  }

  ~RvizPainter() {
    _markersPublisher.shutdown();
  }

  //TODO: void update(Marker) { Marker.update() };

  void draw(PainterObjectBase &obj, const Pose &pose) {
    Marker &m = obj.drawMarker();
    m.pose = pose;
    _markersPublisher.publish(m);
  }

  void draw(RvizArrow &obj, const Pose &pose) {
    Marker &m = obj.drawMarker();
    m.pose = pose;
    _markersPublisher.publish(m);
  }

  void draw(RvizArrow &obj, const tf2::Vector3 &position, const tf2::Quaternion &orientation) {
    Pose pose;
    tf2::toMsg(position, pose.position);
    pose.orientation = tf2::toMsg(orientation);

    draw(obj, pose);
  }

  void draw(RvizLine &obj, const Point &p1, const Point &p2) {
    Marker &m = obj.drawMarker();
    m.points.push_back(p1);
    m.points.push_back(p2);
    
    _markersPublisher.publish(m);
  }

  void draw(RvizLine &obj, const tf2::Vector3 &p1, const tf2::Vector3 &p2) {
    Point p1Msg, p2Msg;
    tf2::toMsg(p1, p1Msg);
    tf2::toMsg(p2, p2Msg);

    draw(obj, p1Msg, p2Msg);
  }

  void draw(RvizLineStrip &obj, const std::list<tf2::Vector3> &points) {
    Marker &m = obj.drawMarker();
    Point pMsg;
    m.points.clear();
    for (auto &point : points) {
      tf2::toMsg(point, pMsg);
      m.points.push_back(pMsg);
    }
    
    _markersPublisher.publish(m);
  }

  void draw(RivzPoints &obj, const std::list<tf2::Vector3> &points) {
    Marker &m = obj.drawMarker();
    Point pMsg;
    m.points.clear();
    for (auto &point : points)
    {
      tf2::toMsg(point, pMsg);
      m.points.push_back(pMsg);
    }
    
    _markersPublisher.publish(m);
  }

  void draw(RvizPosition &obj, const tf2::Vector3 &position) {
    Marker &m = obj.drawMarker();
    tf2::toMsg(position, m.pose.position);

    _markersPublisher.publish(m);
  }
};

#endif // _PUZLLE_ROS_RVIZ_PAINTER_H_
