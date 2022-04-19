#pragma once
#include <ceres/ceres.h>

class IMUFactorScale : public ceres::LossFunction
{
public:
  explicit IMUFactorScale(double scale) : scale_(scale) {}

  virtual void Evaluate(double s, double rho[3]) const
  {
    rho[0] = scale_;
    rho[0] = 0.0;
    rho[0] = 0.0;
  }

private:
  const double scale_;
};

