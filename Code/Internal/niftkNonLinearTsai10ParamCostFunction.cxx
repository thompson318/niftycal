/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai10ParamCostFunction.h"
#include <niftkMatrixUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsai10ParamCostFunction::NonLinearTsai10ParamCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearTsai10ParamCostFunction::~NonLinearTsai10ParamCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearTsai10ParamCostFunction::MeasureType
NonLinearTsai10ParamCostFunction::InternalGetValue(const ParametersType& parameters) const
{
  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  cv::Mat rvec = cvCreateMat(1, 3, CV_64FC1);
  rvec.at<double>(0, 0) = parameters[0];
  rvec.at<double>(0, 1) = parameters[1];
  rvec.at<double>(0, 2) = parameters[2];

  cv::Mat tvec = cvCreateMat(1, 3, CV_64FC1);
  tvec.at<double>(0, 0) = parameters[3];
  tvec.at<double>(0, 1) = parameters[4];
  tvec.at<double>(0, 2) = parameters[5];

  cv::Matx44d extrinsic = niftk::RodriguesToMatrix(rvec, tvec);

  cv::Mat intrinsic = cvCreateMat(3, 3, CV_64FC1);
  intrinsic.at<double>(0, 0) = parameters[6];
  intrinsic.at<double>(0, 1) = 0;
  intrinsic.at<double>(0, 2) = parameters[7];
  intrinsic.at<double>(1, 0) = 0;
  intrinsic.at<double>(1, 1) = parameters[6];
  intrinsic.at<double>(1, 2) = parameters[8];
  intrinsic.at<double>(2, 0) = 0;
  intrinsic.at<double>(2, 1) = 0;
  intrinsic.at<double>(2, 2) = 1;

  cv::Mat distortion = cvCreateMat(1, 4, CV_64FC1);
  distortion.at<double>(0, 0) = parameters[9];
  distortion.at<double>(0, 1) = 0;
  distortion.at<double>(0, 2) = 0;
  distortion.at<double>(0, 3) = 0;

  this->ComputeErrorValues(*m_Model, *(m_Points->begin()), extrinsic, intrinsic, distortion, result);

  return result;
}

} // end namespace
