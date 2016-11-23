/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai10ParamCostFunction_h
#define niftkNonLinearTsai10ParamCostFunction_h

#include "niftkNonLinearCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai10ParamCostFunction
* \brief Cost function, to optimise 2D reprojection error over Rx, Ry, Rz, Tx, Ty, Tz, f, k, Cx and Cy.
* \see niftk::NonLinearTsai10ParamOptimiser
*/
class NonLinearTsai10ParamCostFunction : public niftk::NonLinearCostFunction
{

public:

  typedef NonLinearTsai10ParamCostFunction Self;
  typedef NonLinearCostFunction            Superclass;
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType       ParametersType;
  typedef Superclass::DerivativeType       DerivativeType;
  typedef Superclass::MeasureType          MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearTsai10ParamCostFunction(); // deliberately protected.
  virtual ~NonLinearTsai10ParamCostFunction(); // deliberately protected.

  NonLinearTsai10ParamCostFunction(const NonLinearTsai10ParamCostFunction&); // deliberately not implemented
  NonLinearTsai10ParamCostFunction& operator=(const NonLinearTsai10ParamCostFunction&); // deliberately not implemented

};

} // end namespace

#endif
