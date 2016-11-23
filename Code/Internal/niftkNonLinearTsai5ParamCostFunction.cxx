/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai5ParamCostFunction.h"
#include <niftkMatrixUtilities.h>

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

  cv::Mat tvec = cvCreateMat ( 1, 3, CV_64FC1 );
  tvec.at<double>(0, 0) = Tx;
  tvec.at<double>(0, 1) = Ty;
  tvec.at<double>(0, 2) = parameters[0]; // Tz

  cv::Matx44d extrinsic = niftk::RodriguesToMatrix(rvec, tvec);

  cv::Mat distortion = cvCreateMat ( 1, 4, CV_64FC1 );
  distortion.at<double>(0, 0) = parameters[2]; // k1
  distortion.at<double>(0, 1) = 0;
  distortion.at<double>(0, 2) = 0;
  distortion.at<double>(0, 3) = 0;

  cv::Mat intrinsic = cvCreateMat ( 3, 3, CV_64FC1 );
  intrinsic.at<double>(0, 0) = parameters[1];  // f
  intrinsic.at<double>(0, 1) = 0;
  intrinsic.at<double>(0, 2) = parameters[3];  // Cx
  intrinsic.at<double>(1, 0) = 0;
  intrinsic.at<double>(1, 1) = parameters[1];  // f
  intrinsic.at<double>(1, 2) = parameters[4];  // Cy
  intrinsic.at<double>(2, 0) = 0;
  intrinsic.at<double>(2, 1) = 0;
  intrinsic.at<double>(2, 2) = 1;

  this->ComputeErrorValues(*m_Model, *(m_Points->begin()), extrinsic, intrinsic, distortion, result);

  return result;
}

} // end namespace
