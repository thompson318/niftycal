/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai2ParamCostFunction_h
#define niftkNonLinearTsai2ParamCostFunction_h

#include "niftkNonLinearTsaiCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai2ParamCostFunction
* \brief Cost function, to optimise 2D projection error over Cx and Cy.
* \see niftk::NonLinearTsai2ParamOptimiser
*/
class NonLinearTsai2ParamCostFunction : public niftk::NonLinearTsaiCostFunction
{

public:

  typedef NonLinearTsai2ParamCostFunction Self;
  typedef NonLinearTsaiCostFunction       Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType      ParametersType;
  typedef Superclass::DerivativeType      DerivativeType;
  typedef Superclass::MeasureType         MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearTsai2ParamCostFunction(); // deliberately protected.
  virtual ~NonLinearTsai2ParamCostFunction(); // deliberately protected.

  NonLinearTsai2ParamCostFunction(const NonLinearTsai2ParamCostFunction&); // deliberately not implemented
  NonLinearTsai2ParamCostFunction& operator=(const NonLinearTsai2ParamCostFunction&); // deliberately not implemented

};

} // end namespace

#endif
