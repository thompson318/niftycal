/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereo2DCostFunction_h
#define niftkNonLinearStereo2DCostFunction_h

#include "niftkNonLinearStereoCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereo2DCostFunction
* \brief Base class for non-linear cost functions used to optimise 2D reprojection error.
*/
class NonLinearStereo2DCostFunction : public NonLinearStereoCostFunction
{

public:

  typedef NonLinearStereo2DCostFunction Self;
  typedef NonLinearStereoCostFunction   Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;

  typedef Superclass::ParametersType    ParametersType;
  typedef Superclass::DerivativeType    DerivativeType;
  typedef Superclass::MeasureType       MeasureType;

  virtual unsigned int GetNumberOfValues(void) const ITK_OVERRIDE;

protected:

  NonLinearStereo2DCostFunction();
  virtual ~NonLinearStereo2DCostFunction();

  NonLinearStereo2DCostFunction(const NonLinearStereo2DCostFunction&); // Purposefully not implemented.
  NonLinearStereo2DCostFunction& operator=(const NonLinearStereo2DCostFunction&); // Purposefully not implemented.

};

} // end namespace

#endif
