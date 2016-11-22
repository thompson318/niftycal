/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai3ParamCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

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
void NonLinearTsai3ParamCostFunction::SetDistortion(const cv::Mat* const distortion)
{
  m_Distortion = const_cast<cv::Mat*>(distortion);
  this->Modified();
}


//-----------------------------------------------------------------------------
NonLinearTsai3ParamCostFunction::MeasureType
NonLinearTsai3ParamCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  std::list<niftk::PointSet>::const_iterator iter = m_Points->begin();

  unsigned int totalPointCounter = 0;
  std::vector<cv::Point2f> observed(iter->size());
  std::vector<cv::Point2f> projected(iter->size());
  std::vector<niftk::NiftyCalIdType> ids(iter->size());

  cv::Matx44d extrinsic = *m_Extrinsic;
  cv::Mat intrinsic = m_Intrinsic->clone();
  cv::Mat distortion = m_Distortion->clone();

  extrinsic(2, 3) = parameters[0];             // Tz
  intrinsic.at<double>(0, 0) = parameters[1];  // f
  intrinsic.at<double>(1, 1) = parameters[1];  // f
  distortion.at<double>(0, 0) = parameters[2]; // k1


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
