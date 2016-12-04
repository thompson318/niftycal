/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoCameraCalibration3DCostFunction.h"
#include "niftkCalibrationUtilities_p.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration3DCostFunction::NonLinearStereoCameraCalibration3DCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration3DCostFunction::~NonLinearStereoCameraCalibration3DCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration3DCostFunction::MeasureType
NonLinearStereoCameraCalibration3DCostFunction::InternalGetValue(const ParametersType& parameters) const
{
  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  niftk::ComputeStereoReconstructionErrors(m_Model, m_Points, m_RightHandPoints, parameters, result);
  return result;
}

} // end namespace
