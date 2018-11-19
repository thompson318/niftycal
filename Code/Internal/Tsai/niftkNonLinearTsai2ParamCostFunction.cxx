/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai2ParamCostFunction.h"
#include <Internal/niftkCalibrationUtilities_p.h>
#include <niftkMatrixUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsai2ParamCostFunction::NonLinearTsai2ParamCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearTsai2ParamCostFunction::~NonLinearTsai2ParamCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearTsai2ParamCostFunction::MeasureType
NonLinearTsai2ParamCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  cv::Mat rvec = cv::Mat::zeros ( 1, 3, CV_64FC1 );
  cv::Mat tvec = cv::Mat::zeros ( 1, 3, CV_64FC1 );
  niftk::MatrixToRodrigues(*m_Extrinsic, rvec, tvec);

  cv::Point2d imageCentre;
  imageCentre.x = parameters[0];  // Cx
  imageCentre.y = parameters[1];  // Cy

  ParametersType internalParameters;
  internalParameters.SetSize(  4 // intrinsic
                             + 5 // distortion
                             + 6 // extrinsic
                            );

  internalParameters[0] = (*m_Intrinsic).at<double>(0, 0) * m_Sx;
  internalParameters[1] = (*m_Intrinsic).at<double>(1, 1);
  internalParameters[2] = parameters[0];  // Cx
  internalParameters[3] = parameters[1];  // Cy
  internalParameters[4] = m_K1;
  internalParameters[5] = 0;
  internalParameters[6] = 0;
  internalParameters[7] = 0;
  internalParameters[8] = 0;
  internalParameters[9]  = rvec.at<double>(0, 0); // R1
  internalParameters[10] = rvec.at<double>(0, 1); // R2
  internalParameters[11] = rvec.at<double>(0, 2); // R3
  internalParameters[12] = tvec.at<double>(0, 0); // T1
  internalParameters[13] = tvec.at<double>(0, 1); // T2
  internalParameters[14] = tvec.at<double>(0, 2); // T3

  niftk::ComputeMonoProjectionErrors(m_Model, m_Points, internalParameters, result);
  return result;
}

} // end namespace
