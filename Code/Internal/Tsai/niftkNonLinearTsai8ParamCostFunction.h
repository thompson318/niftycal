/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai8ParamCostFunction_h
#define niftkNonLinearTsai8ParamCostFunction_h

#include "niftkNonLinearTsaiCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai8ParamCostFunction
* \brief Cost function, to optimise 2D projection error over R1, R2, R3 (Rodrigues), Tx, Ty, Tz, f and k.
* \see niftk::NonLinearTsai8ParamOptimiser
*/
class NonLinearTsai8ParamCostFunction : public niftk::NonLinearTsaiCostFunction
{

public:

  typedef NonLinearTsai8ParamCostFunction Self;
  typedef NonLinearMonoCostFunction       Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType      ParametersType;
  typedef Superclass::DerivativeType      DerivativeType;
  typedef Superclass::MeasureType         MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearTsai8ParamCostFunction(); // deliberately protected.
  virtual ~NonLinearTsai8ParamCostFunction(); // deliberately protected.

  NonLinearTsai8ParamCostFunction(const NonLinearTsai8ParamCostFunction&); // deliberately not implemented
  NonLinearTsai8ParamCostFunction& operator=(const NonLinearTsai8ParamCostFunction&); // deliberately not implemented

};

} // end namespace

#endif
