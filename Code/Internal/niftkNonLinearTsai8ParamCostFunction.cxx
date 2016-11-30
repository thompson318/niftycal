/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai8ParamCostFunction.h"
#include "niftkCalibrationUtilities_p.h"

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

  ParametersType internalParameters;
  internalParameters.SetSize(4   // intrinsic
                             + 5 // distortion
                             + 6 // extrinsic
                            );

  internalParameters[0] = parameters[6];  // f
  internalParameters[1] = parameters[6];  // f
  internalParameters[2] = (*m_Intrinsic).at<double>(0, 2);
  internalParameters[3] = (*m_Intrinsic).at<double>(1, 2);
  internalParameters[4] = parameters[7]; // k1
  internalParameters[5] = 0;
  internalParameters[6] = 0;
  internalParameters[7] = 0;
  internalParameters[8] = 0;
  internalParameters[9]  = parameters[0]; // R1
  internalParameters[10] = parameters[1]; // R2
  internalParameters[11] = parameters[2]; // R3
  internalParameters[12] = parameters[3]; // Tx
  internalParameters[13] = parameters[4]; // Ty
  internalParameters[14] = parameters[5]; // Tz

  niftk::ComputeMonoProjectionErrors(m_Model, m_Points, internalParameters, result);
  return result;
}

} // end namespace
