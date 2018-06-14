/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereo2DCostFunction.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereo2DCostFunction::NonLinearStereo2DCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearStereo2DCostFunction::~NonLinearStereo2DCostFunction()
{
}


//-----------------------------------------------------------------------------
unsigned int NonLinearStereo2DCostFunction::GetNumberOfValues(void) const
{
  return (NonLinearCostFunction::GetNumberOfValues()
          + NonLinearStereoCostFunction::GetNumberOfRightHandValues())
          * 2; // multiply by 2 for dx, dy.

  /* Tried this for epi-polar lines.
  return ((NonLinearCostFunction::GetNumberOfValues() + NonLinearStereoCostFunction::GetNumberOfRightHandValues()) * 2.0
          + (this->GetNumberOfTriangulatablePoints() * 2.0)
         );
  */
}

} // end namespace
