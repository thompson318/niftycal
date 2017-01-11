/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoCameraCalibration2DCostFunction.h"
#include "niftkCalibrationUtilities_p.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration2DCostFunction::NonLinearStereoCameraCalibration2DCostFunction()
: m_OptimiseIntrinsics(false)
, m_Optimise2DOFStereo(false)
{
  m_AxisOfRotation = 0;
  m_TranslationVector = 0;
}


//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration2DCostFunction::~NonLinearStereoCameraCalibration2DCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration2DCostFunction::MeasureType
NonLinearStereoCameraCalibration2DCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  if (m_Points->size() != m_RightHandPoints->size())
  {
    niftkNiftyCalThrow() << "Different number of left and right point sets.";
  }

  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  // First work out number of DOF.
  int numberOfParameters = parameters.GetSize() + 5 + 5; // add 5 distortion params for left and right.

  if (m_Optimise2DOFStereo)  // In this case, the l2r transform is an angle (rad), and a translation (mm).
  {
    numberOfParameters += 4; // So we need to add 4DOF for a total of 6DOF.
  }
  if (!m_OptimiseIntrinsics) // In this case, the input doesn't contain intrinsic params.
  {
    numberOfParameters += 8; // So we need to add 4DOF for left and right.
  }

  ParametersType internalParameters;
  internalParameters.SetSize(numberOfParameters);

  unsigned int internalParameterCounter = 0;
  unsigned int externalParameterCounter = 0;

  if (m_OptimiseIntrinsics)
  {
    for (int i = 0; i < 4; i++)
    {
      internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
    }
  }
  else
  {
    internalParameters[internalParameterCounter++] = m_Intrinsic->at<double>(0, 0);
    internalParameters[internalParameterCounter++] = m_Intrinsic->at<double>(1, 1);
    internalParameters[internalParameterCounter++] = m_Intrinsic->at<double>(0, 2);
    internalParameters[internalParameterCounter++] = m_Intrinsic->at<double>(1, 2);
  }
  for (int i = 0; i < 5; i++)
  {
    internalParameters[internalParameterCounter++] = m_Distortion->at<double>(0, i);
  }
  if (m_OptimiseIntrinsics)
  {
    for (int i = 0; i < 4; i++)
    {
      internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
    }
  }
  else
  {
    internalParameters[internalParameterCounter++] = m_RightIntrinsic->at<double>(0, 0);
    internalParameters[internalParameterCounter++] = m_RightIntrinsic->at<double>(1, 1);
    internalParameters[internalParameterCounter++] = m_RightIntrinsic->at<double>(0, 2);
    internalParameters[internalParameterCounter++] = m_RightIntrinsic->at<double>(1, 2);
  }
  for (int i = 0; i < 5; i++)
  {
    internalParameters[internalParameterCounter++] = m_RightDistortion->at<double>(0, i);
  }

  if (m_Optimise2DOFStereo)
  {
    cv::Matx14d axisAngle;
    axisAngle(0, 0) = m_AxisOfRotation[0];
    axisAngle(0, 1) = m_AxisOfRotation[1];
    axisAngle(0, 2) = m_AxisOfRotation[2];
    axisAngle(0, 3) = parameters[externalParameterCounter++];

    cv::Mat rodrigues = niftk::AxisAngleToRodrigues(axisAngle);
    internalParameters[internalParameterCounter++] = rodrigues.at<double>(0, 0);
    internalParameters[internalParameterCounter++] = rodrigues.at<double>(0, 1);
    internalParameters[internalParameterCounter++] = rodrigues.at<double>(0, 2);

    double translationInMillimetres = parameters[externalParameterCounter++];
    internalParameters[internalParameterCounter++] = m_TranslationVector[0] * translationInMillimetres;
    internalParameters[internalParameterCounter++] = m_TranslationVector[1] * translationInMillimetres;
    internalParameters[internalParameterCounter++] = m_TranslationVector[2] * translationInMillimetres;
  }
  else
  {
    for (int i = 0; i < 6; i++)
    {
      internalParameters[internalParameterCounter++] = parameters[externalParameterCounter++];
    }
  }

  // Add rest of parameters = 6DOF for each view.
  for (unsigned int i = externalParameterCounter; i < parameters.GetSize(); i++)
  {
    internalParameters[internalParameterCounter++] = parameters[i];
  }

  niftk::ComputeStereoProjectionErrors(m_Model, m_Points, m_RightHandPoints, internalParameters, result);

  return result;
}

} // end namespace
