/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereo3DCostFunction.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereo3DCostFunction::NonLinearStereo3DCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearStereo3DCostFunction::~NonLinearStereo3DCostFunction()
{
}


//-----------------------------------------------------------------------------
unsigned int NonLinearStereo3DCostFunction::GetNumberOfValues(void) const
{
  return this->GetNumberOfTriangulatablePoints() * 3; // for dx, dy, dz in 3D.
}

} // end namespace
