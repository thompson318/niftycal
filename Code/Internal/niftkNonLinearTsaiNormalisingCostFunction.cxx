/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsaiNormalisingCostFunction.h"
#include "Internal/niftkTsaiUtilities_p.h"
#include "niftkPointUtilities.h"
#include "niftkMatrixUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsaiNormalisingCostFunction::NonLinearTsaiNormalisingCostFunction()
: m_DxPrime(1)
, m_Sx(1)
, m_SensorDimensions(1, 1)
{
}


//-----------------------------------------------------------------------------
NonLinearTsaiNormalisingCostFunction::~NonLinearTsaiNormalisingCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearTsaiNormalisingCostFunction::SetCameraConstants(const double& dxPrime, const cv::Point2d& sensorDimensions, const double& sx)
{
  m_DxPrime = dxPrime;
  m_SensorDimensions = sensorDimensions;
  m_Sx = sx;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsaiNormalisingCostFunction::ComputeRTxAndTy(const niftk::Model3D& model,
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

  niftk::CalculateTxAndTy(points3D, points2D, X, Tx, Ty);

  double f = 0;
  double Tz = 0;
  cv::Mat R = cvCreateMat ( 3, 3, CV_64FC1 );
  niftk::CalculateRWithApproxTzAndF(points3D, points2D, m_SensorDimensions, Ty, X, R, Tz, f);

  cv::Rodrigues(R, rvec);
}

} // end namespace
