/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMaltiHandEyeCostFunction_h
#define niftkNonLinearMaltiHandEyeCostFunction_h

#include "niftkNonLinearMonoCostFunction.h"

namespace niftk
{

/**
* \class NonLinearMaltiHandEyeCostFunction
* \brief Computes cost as RMS re-projection error for intrinsic, hand-eye and model-to-world
* optimisation as per <a href="http://dx.doi.org/10.1002/rcs.1478">Malti 2013</a>.
*
* \see niftk::NonLinearMaltiHandEyeOptimiser
*/
class NonLinearMaltiHandEyeCostFunction : public NonLinearMonoCostFunction
{

public:

  typedef NonLinearMaltiHandEyeCostFunction Self;
  typedef NonLinearMonoCostFunction         Superclass;
  typedef itk::SmartPointer<Self>           Pointer;
  typedef itk::SmartPointer<const Self>     ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType        ParametersType;
  typedef Superclass::DerivativeType        DerivativeType;
  typedef Superclass::MeasureType           MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearMaltiHandEyeCostFunction();
  virtual ~NonLinearMaltiHandEyeCostFunction();

  NonLinearMaltiHandEyeCostFunction(const NonLinearMaltiHandEyeCostFunction&);
  NonLinearMaltiHandEyeCostFunction& operator=(const NonLinearMaltiHandEyeCostFunction&);

};

} // end namespace

#endif
