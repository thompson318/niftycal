/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai5ParamCostFunction.h"
#include "niftkCalibrationUtilities_p.h"
#include "niftkTsaiUtilities_p.h"
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsai5ParamCostFunction::NonLinearTsai5ParamCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearTsai5ParamCostFunction::~NonLinearTsai5ParamCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearTsai5ParamCostFunction::SetCameraConstants(const double& dxPrime, const cv::Point2d& sensorDimensions, const double& sx)
{
  m_DxPrime = dxPrime;
  m_SensorDimensions = sensorDimensions;
  m_Sx = sx;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai5ParamCostFunction::ComputeRTxAndTy(const niftk::Model3D& model,
                                                      const niftk::PointSet& points,
                                                      const cv::Point2d& imageCentre,
                                                      cv::Mat& rvec,
                                                      double& Tx,
                                                      double& Ty
                                                     ) const
{
  std::vector<cv::Point3d> points3D;
  std::vector<cv::Point2d> points2D;
  std::vector<niftk::NiftyCalIdType> ids;

  niftk::ExtractCommonPoints(model, points, points3D, points2D, ids);

  points2D = niftk::NormalisePoints(points2D, m_DxPrime, imageCentre, m_SensorDimensions, m_Sx);

  cv::Mat X = niftk::CalculateEquation10(points3D, points2D);
  double TySquared = niftk::CalculateTySquaredForCoplanar(X);

  niftk::CalculateTxAndTy(points3D, points2D, X, TySquared, Tx, Ty);

  double f = 0;
  double Tz = 0;
  cv::Mat R = cvCreateMat ( 3, 3, CV_64FC1 );

  niftk::CalculateRForCoplanar(X, Ty, R);
  niftk::CalculateRWithFAndTz(points3D, points2D, m_SensorDimensions, Ty, R, Tz, f);

  cv::Rodrigues(R, rvec);
}


//-----------------------------------------------------------------------------
NonLinearTsai5ParamCostFunction::MeasureType
NonLinearTsai5ParamCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  cv::Point2d imageCentre;
  imageCentre.x = parameters[3];  // Cx
  imageCentre.y = parameters[4];  // Cy

  double Tx = 0;
  double Ty = 0;
  cv::Mat rvec = cvCreateMat ( 1, 3, CV_64FC1 );

  this->ComputeRTxAndTy(*m_Model, *(m_Points->begin()), imageCentre, rvec, Tx, Ty);

  ParametersType internalParameters;
  internalParameters.SetSize(4   // intrinsic
                             + 5 // distortion
                             + 6 // extrinsic
                            );

  internalParameters[0] = parameters[1];  // f
  internalParameters[1] = parameters[1];  // f
  internalParameters[2] = parameters[3];  // Cx
  internalParameters[3] = parameters[4];  // Cy
  internalParameters[4] = parameters[2];  // k1
  internalParameters[5] = 0;
  internalParameters[6] = 0;
  internalParameters[7] = 0;
  internalParameters[8] = 0;
  internalParameters[9]  = rvec.at<double>(0, 0); // R1
  internalParameters[10] = rvec.at<double>(0, 1); // R2
  internalParameters[11] = rvec.at<double>(0, 2); // R3
  internalParameters[12] = Tx;
  internalParameters[13] = Ty;
  internalParameters[14] = parameters[0]; // Tz

  niftk::ComputeMonoProjectionErrors(m_Model, m_Points, internalParameters, result);
  return result;
}

} // end namespace
