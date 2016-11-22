/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai8ParamCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsai8ParamCostFunction::NonLinearTsai8ParamCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearTsai8ParamCostFunction::~NonLinearTsai8ParamCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearTsai8ParamCostFunction::SetIntrinsic(const cv::Mat* const intrinsic)
{
  m_Intrinsic = const_cast<cv::Mat*>(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
NonLinearTsai8ParamCostFunction::MeasureType
NonLinearTsai8ParamCostFunction::InternalGetValue(const ParametersType& parameters) const
{
  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  std::list<niftk::PointSet>::const_iterator iter = m_Points->begin();

  unsigned int totalPointCounter = 0;
  std::vector<cv::Point2f> observed(iter->size());
  std::vector<cv::Point2f> projected(iter->size());
  std::vector<niftk::NiftyCalIdType> ids(iter->size());

  cv::Mat rvec = cvCreateMat(1, 3, CV_64FC1);
  rvec.at<double>(0, 0) = parameters[0];
  rvec.at<double>(0, 1) = parameters[1];
  rvec.at<double>(0, 2) = parameters[2];

  cv::Mat tvec = cvCreateMat(1, 3, CV_64FC1);
  tvec.at<double>(0, 0) = parameters[3];
  tvec.at<double>(0, 1) = parameters[4];
  tvec.at<double>(0, 2) = parameters[5];

  cv::Matx44d extrinsic = niftk::RodriguesToMatrix(rvec, tvec);

  cv::Mat intrinsic = m_Intrinsic->clone();
  intrinsic.at<double>(0, 0) = parameters[6];
  intrinsic.at<double>(1, 1) = parameters[6];

  cv::Mat distortion = cvCreateMat(1, 4, CV_64FC1);
  distortion.at<double>(0, 0) = parameters[7];
  distortion.at<double>(0, 1) = 0;
  distortion.at<double>(0, 2) = 0;
  distortion.at<double>(0, 3) = 0;

  niftk::ProjectMatchingPoints(*m_Model,
                               *iter,
                               extrinsic,
                               intrinsic,
                               distortion,
                               observed,
                               projected,
                               ids
                              );

  for (unsigned int i = 0; i < observed.size(); i++)
  {
    result[totalPointCounter++] = (observed[i].x - projected[i].x);
    result[totalPointCounter++] = (observed[i].y - projected[i].y);
  }

  return result;
}

} // end namespace
