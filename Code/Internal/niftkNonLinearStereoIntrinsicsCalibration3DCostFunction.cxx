/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoIntrinsicsCalibration3DCostFunction.h"
#include "niftkCalibrationUtilities_p.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibration3DCostFunction::NonLinearStereoIntrinsicsCalibration3DCostFunction()
: m_LeftDistortion(nullptr)
, m_RightDistortion(nullptr)
{
}


//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibration3DCostFunction::~NonLinearStereoIntrinsicsCalibration3DCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibration3DCostFunction::SetDistortionParameters(cv::Mat* const leftDistortion,
                                                                               cv::Mat* const rightDistortion
                                                                              )
{
  if (leftDistortion == nullptr)
  {
    niftkNiftyCalThrow() << "Null left distortion parameters.";
  }

  if (rightDistortion == nullptr)
  {
    niftkNiftyCalThrow() << "Null right distortion parameters.";
  }

  if (leftDistortion->rows != 1 || leftDistortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Left distortion vector should be a 1x5 vector.";
  }

  if (rightDistortion->rows != 1 || rightDistortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Right distortion vector should be a 1x5 vector.";
  }

  m_LeftDistortion = leftDistortion;
  m_RightDistortion = rightDistortion;
  this->Modified();
}


//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibration3DCostFunction::MeasureType
NonLinearStereoIntrinsicsCalibration3DCostFunction::InternalGetValue(const ParametersType& parameters) const
{
  if (m_Points->size() != m_RightHandPoints->size())
  {
    niftkNiftyCalThrow() << "Different number of left and right point sets.";
  }

  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  ParametersType internalParameters;
  internalParameters.SetSize(  4                    // left intrinsic
                             + 5                    // left distortion
                             + 4                    // right intrinsic
                             + 5                    // right distortion
                             + 6                    // left to right 6 DOF
                             + 6 * m_Points->size() // 6DOF per view
                            );

  internalParameters[0] = parameters[0];
  internalParameters[1] = parameters[1];
  internalParameters[2] = parameters[2];
  internalParameters[3] = parameters[3];
  internalParameters[4] = m_LeftDistortion->at<double>(0, 0);
  internalParameters[5] = m_LeftDistortion->at<double>(0, 1);
  internalParameters[6] = m_LeftDistortion->at<double>(0, 2);
  internalParameters[7] = m_LeftDistortion->at<double>(0, 3);
  internalParameters[8] = m_LeftDistortion->at<double>(0, 4);
  internalParameters[9]  = parameters[4];
  internalParameters[10] = parameters[5];
  internalParameters[11] = parameters[6];
  internalParameters[12] = parameters[7];
  internalParameters[13] = m_RightDistortion->at<double>(0, 0);
  internalParameters[14] = m_RightDistortion->at<double>(0, 1);
  internalParameters[15] = m_RightDistortion->at<double>(0, 2);
  internalParameters[16] = m_RightDistortion->at<double>(0, 3);
  internalParameters[17] = m_RightDistortion->at<double>(0, 4);

  cv::Mat leftToRightRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  cv::Rodrigues(*m_LeftToRightRotationMatrix, leftToRightRotationVector);

  internalParameters[18] = leftToRightRotationVector.at<double>(0, 0);
  internalParameters[19] = leftToRightRotationVector.at<double>(0, 1);
  internalParameters[20] = leftToRightRotationVector.at<double>(0, 2);

  internalParameters[21] = m_LeftToRightTranslationVector->at<double>(0, 0);
  internalParameters[22] = m_LeftToRightTranslationVector->at<double>(1, 0);
  internalParameters[23] = m_LeftToRightTranslationVector->at<double>(2, 0);

  unsigned int parameterCounter = 24;
  for (unsigned int i = 0; i < m_Points->size(); i++)
  {
    internalParameters[parameterCounter++] = (*m_RvecsLeft)[i].at<double>(0, 0);
    internalParameters[parameterCounter++] = (*m_RvecsLeft)[i].at<double>(0, 1);
    internalParameters[parameterCounter++] = (*m_RvecsLeft)[i].at<double>(0, 2);
    internalParameters[parameterCounter++] = (*m_TvecsLeft)[i].at<double>(0, 0);
    internalParameters[parameterCounter++] = (*m_TvecsLeft)[i].at<double>(0, 1);
    internalParameters[parameterCounter++] = (*m_TvecsLeft)[i].at<double>(0, 2);
  }

  niftk::ComputeStereoReconstructionErrors(m_Model, m_Points, m_RightHandPoints, internalParameters, result);
  return result;
}

} // end namespace
