#ifndef QUINTIC_SPLINES_INCLUDE_QUINTIC_SPLINES_POSITION_SPLINE_H_
#define QUINTIC_SPLINES_INCLUDE_QUINTIC_SPLINES_POSITION_SPLINE_H_

#include <tf2/LinearMath/Vector3.h>

#include <quintic_splines/smooth_spline.hpp>
#include <quintic_splines/spline_container.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace quintic_splines {

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
}  // namespace quintic_splines
#endif  // QUINTIC_SPLINES_INCLUDE_QUINTIC_SPLINES_POSITION_SPLINE_H_
