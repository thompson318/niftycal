/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsaiCostFunction.h"
#include "niftkCalibrationUtilities_p.h"
#include <niftkMatrixUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsaiCostFunction::NonLinearTsaiCostFunction()
: m_Extrinsic(nullptr)
, m_Intrinsic(nullptr)
, m_Sx(1)
, m_K1(0)
{
}


//-----------------------------------------------------------------------------
NonLinearTsaiCostFunction::~NonLinearTsaiCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearTsaiCostFunction::SetExtrinsic(const cv::Matx44d* extrinsic)
{
  m_Extrinsic = const_cast<cv::Matx44d*>(extrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsaiCostFunction::SetIntrinsic(const cv::Mat* const intrinsic)
{
  m_Intrinsic = const_cast<cv::Mat*>(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsaiCostFunction::SetSx(const double& sx)
{
  m_Sx = sx;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsaiCostFunction::SetK1(const double& k1)
{
  m_K1 = k1;
  this->Modified();
}

} // end namespace
