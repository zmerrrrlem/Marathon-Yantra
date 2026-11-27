from smooth_spline import SmoothSpline

s = SmoothSpline()

s.add_point(0, 0, 0, 0)
s.add_point(1, 1, 0, 1)
s.compute_spline()
print(s.acc(0.5))
