/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearNoIntrinsicsHandEyeCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearNoIntrinsicsHandEyeCostFunction::NonLinearNoIntrinsicsHandEyeCostFunction()
: m_Intrinsic(nullptr)
, m_Distortion(nullptr)
{
}


//-----------------------------------------------------------------------------
NonLinearNoIntrinsicsHandEyeCostFunction::~NonLinearNoIntrinsicsHandEyeCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearNoIntrinsicsHandEyeCostFunction::SetIntrinsic(const cv::Mat* const intrinsic)
{
  if (intrinsic->rows != 3 || intrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Intrinsic matrix should be 3x3, and its ("
                         << intrinsic->cols << ", " << intrinsic->rows << ")";
  }

  m_Intrinsic = const_cast<cv::Mat*>(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearNoIntrinsicsHandEyeCostFunction::SetDistortion(const cv::Mat* const distortion)
{
  if (distortion->rows != 1 || distortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Distortion vector should 1x5, and its ("
                         << distortion->cols << ", " << distortion->rows << ")";
  }

  m_Distortion = const_cast<cv::Mat*>(distortion);
  this->Modified();
}

} // end namespace
