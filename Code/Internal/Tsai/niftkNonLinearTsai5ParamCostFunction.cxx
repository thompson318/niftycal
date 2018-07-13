/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai5ParamCostFunction.h"
#include <Internal/niftkCalibrationUtilities_p.h>
#include <Internal/Tsai/niftkTsaiUtilities_p.h>
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

  cv::Mat rvec = cvCreateMat ( 1, 3, CV_64FC1 );
  cv::Mat tvec = cvCreateMat ( 1, 3, CV_64FC1 );
  niftk::MatrixToRodrigues(*m_Extrinsic, rvec, tvec);

  ParametersType internalParameters;
  internalParameters.SetSize(  4 // intrinsic
                             + 5 // distortion
                             + 6 // extrinsic
                            );

  internalParameters[0] = parameters[1] * m_Sx;  // f * sx
  internalParameters[1] = parameters[1];         // f
  internalParameters[2] = parameters[3];         // Cx
  internalParameters[3] = parameters[4];         // Cy
  internalParameters[4] = parameters[2];         // k1
  internalParameters[5] = 0;
  internalParameters[6] = 0;
  internalParameters[7] = 0;
  internalParameters[8] = 0;
  internalParameters[9]  = rvec.at<double>(0, 0); // R1
  internalParameters[10] = rvec.at<double>(0, 1); // R2
  internalParameters[11] = rvec.at<double>(0, 2); // R3
  internalParameters[12] = tvec.at<double>(0, 0); // T1
  internalParameters[13] = tvec.at<double>(0, 1); // T2
  internalParameters[14] = parameters[0]; // Tz

  niftk::ComputeMonoProjectionErrors(m_Model, m_Points, internalParameters, result);
  return result;
}

} // end namespace
