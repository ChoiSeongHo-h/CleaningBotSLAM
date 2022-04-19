#pragma once
#include <ceres/ceres.h>

class BiasMahalanobis : public ceres::SizedCostFunction<6, 9>
{
public:
  BiasMahalanobis() = delete;
  BiasMahalanobis(double _baMean, double _bgMean, double _baVar, double _bgVar) : 
  baMean(_baMean), bgMean(_bgMean), baVar(_baVar), bgVar(_bgVar)
  {
  }

  bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const override
  {
    double b[6];
    b[0] = parameters[0][3];
    b[1] = parameters[0][4];
    b[2] = parameters[0][5];
    b[3] = parameters[0][6];
    b[4] = parameters[0][7];
    b[5] = parameters[0][8];
    
    double dB[6];
    dB[0] = b[0]-baMean;
    dB[1] = b[1]-baMean;
    dB[2] = b[2]-baMean;
    dB[3] = b[3]-bgMean;
    dB[4] = b[4]-bgMean;
    dB[5] = b[5]-bgMean;

    residuals[0] = pow(dB[0], 2)/baVar;
    residuals[1] = pow(dB[1], 2)/baVar;
    residuals[2] = pow(dB[2], 2)/baVar;
    residuals[3] = pow(dB[3], 2)/bgVar;
    residuals[4] = pow(dB[4], 2)/bgVar;
    residuals[5] = pow(dB[5], 2)/bgVar;

    if (jacobians != nullptr && jacobians[0] != nullptr)
    {
      Eigen::Map<Eigen::Matrix<double, 1, 54, Eigen::RowMajor>> jacobian(jacobians[0]);
      jacobian.setZero();
      jacobians[0][3] = 2*dB[0]/baVar;
      jacobians[0][13] = 2*dB[1]/baVar;
      jacobians[0][23] = 2*dB[2]/baVar;
      jacobians[0][33] = 2*dB[3]/bgVar;
      jacobians[0][43] = 2*dB[4]/bgVar;
      jacobians[0][53] = 2*dB[5]/bgVar;
    }
    return true;
  }

  double baMean;
  double bgMean;
  double baVar;
  double bgVar;
};
