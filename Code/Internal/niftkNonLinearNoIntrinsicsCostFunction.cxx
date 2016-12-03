/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearNoIntrinsicsCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearNoIntrinsicsCostFunction::NonLinearNoIntrinsicsCostFunction()
: m_Intrinsic(nullptr)
, m_Distortion(nullptr)
, m_RightIntrinsic(nullptr)
, m_RightDistortion(nullptr)
{
}


//-----------------------------------------------------------------------------
NonLinearNoIntrinsicsCostFunction::~NonLinearNoIntrinsicsCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearNoIntrinsicsCostFunction::SetIntrinsic(const cv::Mat* const intrinsic)
{
  if (intrinsic->rows != 3 || intrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Intrinsic matrix should be 3x3, and its ("
                         << intrinsic->cols << ", " << intrinsic->rows << ")";
  }

  m_Intrinsic = const_cast<cv::Mat*>(intrinsic);
}


//-----------------------------------------------------------------------------
void NonLinearNoIntrinsicsCostFunction::SetDistortion(const cv::Mat* const distortion)
{
  if (distortion->rows != 1 || distortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Distortion vector should 1x5, and its ("
                         << distortion->cols << ", " << distortion->rows << ")";
  }

  m_Distortion = const_cast<cv::Mat*>(distortion);
}


//-----------------------------------------------------------------------------
void NonLinearNoIntrinsicsCostFunction::SetRightIntrinsic(const cv::Mat* const intrinsic)
{
  if (intrinsic->rows != 3 || intrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Right Intrinsic matrix should be 3x3, and its ("
                         << intrinsic->cols << ", " << intrinsic->rows << ")";
  }

  m_RightIntrinsic = const_cast<cv::Mat*>(intrinsic);
}


//-----------------------------------------------------------------------------
void NonLinearNoIntrinsicsCostFunction::SetRightDistortion(const cv::Mat* const distortion)
{
  if (distortion->rows != 1 || distortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Distortion vector should 1x5, and its ("
                         << distortion->cols << ", " << distortion->rows << ")";
  }

  m_RightDistortion = const_cast<cv::Mat*>(distortion);
}

} // end namespace
