#ifndef CUBIC_SPLINES_INCLUDE_CUBIC_SPLINES_POSITION_SPLINE_H_
#define CUBIC_SPLINES_INCLUDE_CUBIC_SPLINES_POSITION_SPLINE_H_

#include <tf2/LinearMath/Vector3.h>

#include <cubic_splines/smooth_spline.hpp>
#include <cubic_splines/spline_container.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace cubic_splines {

class PositionSpline {
 public:
  geometry_msgs::msg::Point getGeometryMsgPosition(double time);

  tf2::Vector3 getPos(double time);
  tf2::Vector3 getVel(double time);
  tf2::Vector3 getAcc(double time);

  SmoothSpline *x();
  SmoothSpline *y();
  SmoothSpline *z();

 private:
  SmoothSpline x_;
  SmoothSpline y_;
  SmoothSpline z_;
};
}  // namespace cubic_splines
#endif  // CUBIC_SPLINES_INCLUDE_CUBIC_SPLINES_POSITION_SPLINE_H_
