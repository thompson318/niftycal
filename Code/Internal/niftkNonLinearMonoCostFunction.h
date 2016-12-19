/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMonoCostFunction_h
#define niftkNonLinearMonoCostFunction_h

#include "niftkNonLinearCostFunction.h"

namespace niftk
{

/**
* \class NonLinearMonoCostFunction
* \brief Base class for non-linear cost functions used to optimise 2D reprojection error.
*/
class NonLinearMonoCostFunction : public NonLinearCostFunction
{

public:

  typedef NonLinearMonoCostFunction     Self;
  typedef NonLinearCostFunction         Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;

  typedef Superclass::ParametersType    ParametersType;
  typedef Superclass::DerivativeType    DerivativeType;
  typedef Superclass::MeasureType       MeasureType;

  virtual unsigned int GetNumberOfValues(void) const ITK_OVERRIDE;

protected:

  NonLinearMonoCostFunction();
  virtual ~NonLinearMonoCostFunction();

  NonLinearMonoCostFunction(const NonLinearMonoCostFunction&); // Purposefully not implemented.
  NonLinearMonoCostFunction& operator=(const NonLinearMonoCostFunction&); // Purposefully not implemented.

private:

};

} // end namespace

#endif
