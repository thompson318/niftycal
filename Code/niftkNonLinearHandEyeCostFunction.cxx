/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearHandEyeCostFunction.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::NonLinearHandEyeCostFunction()
{

}


//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::~NonLinearHandEyeCostFunction()
{

}


//-----------------------------------------------------------------------------
unsigned int NonLinearHandEyeCostFunction::GetNumberOfValues(void) const
{
  return 0;
}


//-----------------------------------------------------------------------------
unsigned int NonLinearHandEyeCostFunction::GetNumberOfParameters() const
{
  return 0;
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::GetDerivative(const ParametersType& parameters, DerivativeType& derivative ) const
{

}


//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::MeasureType
NonLinearHandEyeCostFunction::GetValue(const ParametersType& parameters ) const
{
  MeasureType result;
  return result;
}


} // end namespace
