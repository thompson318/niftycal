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
: m_OptimiseIntrinsics(true)
, m_Optimise2DOFStereo(false)
{
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

  if (m_OptimiseIntrinsics)
  {
    // (still need to add distortion parameters)

    ParametersType internalParameters;
    internalParameters.SetSize(parameters.GetSize()
                               + 5 // left distortion
                               + 5 // right distortion
                              );

    unsigned int parameterCounter = 0;
    internalParameters[parameterCounter++] = parameters[0];
    internalParameters[parameterCounter++] = parameters[1];
    internalParameters[parameterCounter++] = parameters[2];
    internalParameters[parameterCounter++] = parameters[3];
    internalParameters[parameterCounter++] = m_Distortion->at<double>(0, 0);
    internalParameters[parameterCounter++] = m_Distortion->at<double>(0, 1);
    internalParameters[parameterCounter++] = m_Distortion->at<double>(0, 2);
    internalParameters[parameterCounter++] = m_Distortion->at<double>(0, 3);
    internalParameters[parameterCounter++] = m_Distortion->at<double>(0, 4);
    internalParameters[parameterCounter++] = parameters[4];
    internalParameters[parameterCounter++] = parameters[5];
    internalParameters[parameterCounter++] = parameters[6];
    internalParameters[parameterCounter++] = parameters[7];
    internalParameters[parameterCounter++] = m_RightDistortion->at<double>(0, 0);
    internalParameters[parameterCounter++] = m_RightDistortion->at<double>(0, 1);
    internalParameters[parameterCounter++] = m_RightDistortion->at<double>(0, 2);
    internalParameters[parameterCounter++] = m_RightDistortion->at<double>(0, 3);
    internalParameters[parameterCounter++] = m_RightDistortion->at<double>(0, 4);
    for (unsigned int i = 8; i < parameters.GetSize(); i++)
    {
      internalParameters[parameterCounter++] = parameters[i];
    }

    niftk::ComputeStereoProjectionErrors(m_Model, m_Points, m_RightHandPoints, internalParameters, result);
  }
  else
  {
    ParametersType internalParameters;
    internalParameters.SetSize(parameters.GetSize()
                               + 9 // left intrinsic and distortion
                               + 9 // right intrinsic and distortion
                              );

    unsigned int parameterCounter = 0;
    internalParameters[parameterCounter++] = m_Intrinsic->at<double>(0, 0);
    internalParameters[parameterCounter++] = m_Intrinsic->at<double>(1, 1);
    internalParameters[parameterCounter++] = m_Intrinsic->at<double>(0, 2);
    internalParameters[parameterCounter++] = m_Intrinsic->at<double>(1, 2);
    internalParameters[parameterCounter++] = m_Distortion->at<double>(0, 0);
    internalParameters[parameterCounter++] = m_Distortion->at<double>(0, 1);
    internalParameters[parameterCounter++] = m_Distortion->at<double>(0, 2);
    internalParameters[parameterCounter++] = m_Distortion->at<double>(0, 3);
    internalParameters[parameterCounter++] = m_Distortion->at<double>(0, 4);
    internalParameters[parameterCounter++] = m_RightIntrinsic->at<double>(0, 0);
    internalParameters[parameterCounter++] = m_RightIntrinsic->at<double>(1, 1);
    internalParameters[parameterCounter++] = m_RightIntrinsic->at<double>(0, 2);
    internalParameters[parameterCounter++] = m_RightIntrinsic->at<double>(1, 2);
    internalParameters[parameterCounter++] = m_RightDistortion->at<double>(0, 0);
    internalParameters[parameterCounter++] = m_RightDistortion->at<double>(0, 1);
    internalParameters[parameterCounter++] = m_RightDistortion->at<double>(0, 2);
    internalParameters[parameterCounter++] = m_RightDistortion->at<double>(0, 3);
    internalParameters[parameterCounter++] = m_RightDistortion->at<double>(0, 4);
    for (unsigned int i = 0; i < parameters.GetSize(); i++)
    {
      internalParameters[parameterCounter++] = parameters[i];
    }

    niftk::ComputeStereoProjectionErrors(m_Model, m_Points, m_RightHandPoints, internalParameters, result);
  }
  return result;
}

} // end namespace
