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

  niftk::ComputeStereoProjectionErrors(m_Model, m_Points, m_RightHandPoints, parameters, result);
  return result;
}

} // end namespace
