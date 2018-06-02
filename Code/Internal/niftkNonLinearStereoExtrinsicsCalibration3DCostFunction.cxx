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
: m_OptimiseCameraExtrinsics(true)
, m_OptimiseL2R(true)
{
}


//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibration3DCostFunction::~NonLinearStereoExtrinsicsCalibration3DCostFunction()
{
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

  cv::Mat leftToRightRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  cv::Rodrigues(*m_LeftToRightRotationMatrix, leftToRightRotationVector);

  unsigned int internalParameterCounter = 0;
  unsigned int externalParameterCounter = 0;

  internalParameters[internalParameterCounter++] = m_Intrinsic->at<double>(0, 0);
  internalParameters[internalParameterCounter++] = m_Intrinsic->at<double>(1, 1);
  internalParameters[internalParameterCounter++] = m_Intrinsic->at<double>(0, 2);
  internalParameters[internalParameterCounter++] = m_Intrinsic->at<double>(1, 2);
  internalParameters[internalParameterCounter++] = m_Distortion->at<double>(0, 0);
  internalParameters[internalParameterCounter++] = m_Distortion->at<double>(0, 1);
  internalParameters[internalParameterCounter++] = m_Distortion->at<double>(0, 2);
  internalParameters[internalParameterCounter++] = m_Distortion->at<double>(0, 3);
  internalParameters[internalParameterCounter++] = m_Distortion->at<double>(0, 4);
  internalParameters[internalParameterCounter++]  = m_RightIntrinsic->at<double>(0, 0);
  internalParameters[internalParameterCounter++] = m_RightIntrinsic->at<double>(1, 1);
  internalParameters[internalParameterCounter++] = m_RightIntrinsic->at<double>(0, 2);
  internalParameters[internalParameterCounter++] = m_RightIntrinsic->at<double>(1, 2);
  internalParameters[internalParameterCounter++] = m_RightDistortion->at<double>(0, 0);
  internalParameters[internalParameterCounter++] = m_RightDistortion->at<double>(0, 1);
  internalParameters[internalParameterCounter++] = m_RightDistortion->at<double>(0, 2);
  internalParameters[internalParameterCounter++] = m_RightDistortion->at<double>(0, 3);
  internalParameters[internalParameterCounter++] = m_RightDistortion->at<double>(0, 4);

  if (m_OptimiseL2R)
  {
    internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
    internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
    internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
    internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
    internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
    internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
  }
  else
  {
    internalParameters[internalParameterCounter++] = leftToRightRotationVector.at<double>(0, 0);
    internalParameters[internalParameterCounter++] = leftToRightRotationVector.at<double>(0, 1);
    internalParameters[internalParameterCounter++] = leftToRightRotationVector.at<double>(0, 2);
    internalParameters[internalParameterCounter++] = m_LeftToRightTranslationVector->at<double>(0, 0);
    internalParameters[internalParameterCounter++] = m_LeftToRightTranslationVector->at<double>(1, 0);
    internalParameters[internalParameterCounter++] = m_LeftToRightTranslationVector->at<double>(2, 0);
  }

  if (m_OptimiseCameraExtrinsics)
  {
    for (unsigned int i = 0; i < m_Points->size(); i++)
    {
      internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
      internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
      internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
      internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
      internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
      internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
    }
  }
  else
  {
    for (unsigned int i = 0; i < m_Points->size(); i++)
    {
      internalParameters[internalParameterCounter++] = (*m_RvecsLeft)[i].at<double>(0, 0);
      internalParameters[internalParameterCounter++] = (*m_RvecsLeft)[i].at<double>(0, 1);
      internalParameters[internalParameterCounter++] = (*m_RvecsLeft)[i].at<double>(0, 2);
      internalParameters[internalParameterCounter++] = (*m_TvecsLeft)[i].at<double>(0, 0);
      internalParameters[internalParameterCounter++] = (*m_TvecsLeft)[i].at<double>(0, 1);
      internalParameters[internalParameterCounter++] = (*m_TvecsLeft)[i].at<double>(0, 2);
    }
  }

  niftk::ComputeStereoReconstructionErrors(m_Model, m_Points, m_RightHandPoints, internalParameters, result);
  return result;
}

} // end namespace
