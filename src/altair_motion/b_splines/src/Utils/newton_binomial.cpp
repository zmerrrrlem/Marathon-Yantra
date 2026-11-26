/*
This code is largely based on the original code by Quentin "Leph" Rouxel and Team Rhoban.
The original files can be found at:
https://github.com/Rhoban/model/
*/
#include "b_splines/newton_binomial.hpp"

#include <cmath>

namespace b_splines {

Polynom NewtonBinomial::expandPolynom(double y, unsigned int degree) {
  Combination combination;

  Polynom polynom;
  polynom.getCoefs().resize(degree + 1);

  for (size_t k = 0; k <= degree; k++) {
    polynom.getCoefs()[k] = combination.binomialCoefficient(k, degree) * pow(y, degree - k);
  }

  return polynom;
}

}  // namespace b_splines
