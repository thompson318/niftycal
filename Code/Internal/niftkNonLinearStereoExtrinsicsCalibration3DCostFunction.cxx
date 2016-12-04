/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoExtrinsicsCalibration3DCostFunction.h"
#include "niftkCalibrationUtilities_p.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibration3DCostFunction::NonLinearStereoExtrinsicsCalibration3DCostFunction()
: m_LeftIntrinsic(nullptr)
, m_LeftDistortion(nullptr)
, m_RightIntrinsic(nullptr)
, m_RightDistortion(nullptr)
{
}


//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibration3DCostFunction::~NonLinearStereoExtrinsicsCalibration3DCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibration3DCostFunction::SetIntrinsics(cv::Mat* const leftIntrinsic,
                                                                     cv::Mat* const rightIntrinsic
                                                                    )
{
  if (leftIntrinsic->rows != 3 || leftIntrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Left intrinsic matrix should be 3x3, and its ("
                         << leftIntrinsic->cols << ", " << leftIntrinsic->rows << ")";
  }

  if (rightIntrinsic->rows != 3 || rightIntrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Right intrinsic matrix should be 3x3, and its ("
                         << rightIntrinsic->cols << ", " << rightIntrinsic->rows << ")";
  }

  m_LeftIntrinsic = leftIntrinsic;
  m_RightIntrinsic = rightIntrinsic;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibration3DCostFunction::SetDistortionParameters(cv::Mat* const leftDistortion,
                                                                               cv::Mat* const rightDistortion
                                                                              )
{
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
NonLinearStereoExtrinsicsCalibration3DCostFunction::MeasureType
NonLinearStereoExtrinsicsCalibration3DCostFunction::InternalGetValue(const ParametersType& parameters) const
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

  internalParameters[0] = m_LeftIntrinsic->at<double>(0, 0);
  internalParameters[1] = m_LeftIntrinsic->at<double>(1, 1);
  internalParameters[2] = m_LeftIntrinsic->at<double>(0, 2);
  internalParameters[3] = m_LeftIntrinsic->at<double>(1, 2);
  internalParameters[4] = m_LeftDistortion->at<double>(0, 0);
  internalParameters[5] = m_LeftDistortion->at<double>(0, 1);
  internalParameters[6] = m_LeftDistortion->at<double>(0, 2);
  internalParameters[7] = m_LeftDistortion->at<double>(0, 3);
  internalParameters[8] = m_LeftDistortion->at<double>(0, 4);
  internalParameters[9]  = m_RightIntrinsic->at<double>(0, 0);
  internalParameters[10] = m_RightIntrinsic->at<double>(1, 1);
  internalParameters[11] = m_RightIntrinsic->at<double>(0, 2);
  internalParameters[12] = m_RightIntrinsic->at<double>(1, 2);
  internalParameters[13] = m_RightDistortion->at<double>(0, 0);
  internalParameters[14] = m_RightDistortion->at<double>(0, 1);
  internalParameters[15] = m_RightDistortion->at<double>(0, 2);
  internalParameters[16] = m_RightDistortion->at<double>(0, 3);
  internalParameters[17] = m_RightDistortion->at<double>(0, 4);

  internalParameters[18] = parameters[0];
  internalParameters[19] = parameters[1];
  internalParameters[20] = parameters[2];

  internalParameters[21] = parameters[3];
  internalParameters[22] = parameters[4];
  internalParameters[23] = parameters[5];

  unsigned int internalParameterCounter = 24;
  unsigned int parametersCounter = 6;
  for (unsigned int i = 0; i < m_Points->size(); i++)
  {
    internalParameters[internalParameterCounter++] = parameters[parametersCounter++];
    internalParameters[internalParameterCounter++] = parameters[parametersCounter++];
    internalParameters[internalParameterCounter++] = parameters[parametersCounter++];
    internalParameters[internalParameterCounter++] = parameters[parametersCounter++];
    internalParameters[internalParameterCounter++] = parameters[parametersCounter++];
    internalParameters[internalParameterCounter++] = parameters[parametersCounter++];
  }

  niftk::ComputeStereoReconstructionErrors(m_Model, m_Points, m_RightHandPoints, internalParameters, result);
  return result;
}

} // end namespace
