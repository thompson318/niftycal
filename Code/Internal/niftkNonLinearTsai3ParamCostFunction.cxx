/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai3ParamCostFunction.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsai3ParamCostFunction::NonLinearTsai3ParamCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearTsai3ParamCostFunction::~NonLinearTsai3ParamCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearTsai3ParamCostFunction::SetExtrinsic(const cv::Matx44d* extrinsic)
{
  m_Extrinsic = const_cast<cv::Matx44d*>(extrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai3ParamCostFunction::SetIntrinsic(const cv::Mat* const intrinsic)
{
  m_Intrinsic = const_cast<cv::Mat*>(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
NonLinearTsai3ParamCostFunction::MeasureType
NonLinearTsai3ParamCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  cv::Matx44d extrinsic = *m_Extrinsic;
  extrinsic(2, 3) = parameters[0]; // Tz

  cv::Mat intrinsic = m_Intrinsic->clone();
  intrinsic.at<double>(0, 0) = parameters[1];  // f
  intrinsic.at<double>(1, 1) = parameters[1];  // f

  cv::Mat distortion = cvCreateMat ( 1, 4, CV_64FC1 );
  distortion.at<double>(0, 0) = parameters[2]; // k1
  distortion.at<double>(0, 1) = 0;
  distortion.at<double>(0, 2) = 0;
  distortion.at<double>(0, 3) = 0;

  this->ComputeErrorValues(*m_Model, *(m_Points->begin()), extrinsic, intrinsic, distortion, result);

  return result;
}

} // end namespace
