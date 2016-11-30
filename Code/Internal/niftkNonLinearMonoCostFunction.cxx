/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearMonoCostFunction.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearMonoCostFunction::NonLinearMonoCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearMonoCostFunction::~NonLinearMonoCostFunction()
{
}


//-----------------------------------------------------------------------------
unsigned int NonLinearMonoCostFunction::GetNumberOfValues(void) const
{
  return NonLinearCostFunction::GetNumberOfValues() * 2; // multiply by 2 for dx, dy.
}

} // end namespace
