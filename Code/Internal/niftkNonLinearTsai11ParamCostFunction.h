/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai11ParamCostFunction_h
#define niftkNonLinearTsai11ParamCostFunction_h

#include "niftkNonLinearTsaiCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai11ParamCostFunction
* \brief Cost function, to optimise 2D projection error over R1, R2, R3 (Rodrigues), Tx, Ty, Tz, f, k, Cx, Cy and sx.
* \see niftk::NonLinearTsai11ParamOptimiser
*/
class NonLinearTsai11ParamCostFunction : public niftk::NonLinearTsaiCostFunction
{

public:

  typedef NonLinearTsai11ParamCostFunction Self;
  typedef NonLinearTsaiCostFunction        Superclass;
  typedef itk::SmartPointer<Self>          Pointer;
  typedef itk::SmartPointer<const Self>    ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType       ParametersType;
  typedef Superclass::DerivativeType       DerivativeType;
  typedef Superclass::MeasureType          MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearTsai11ParamCostFunction(); // deliberately protected.
  virtual ~NonLinearTsai11ParamCostFunction(); // deliberately protected.

  NonLinearTsai11ParamCostFunction(const NonLinearTsai11ParamCostFunction&); // deliberately not implemented
  NonLinearTsai11ParamCostFunction& operator=(const NonLinearTsai11ParamCostFunction&); // deliberately not implemented

};

} // end namespace

#endif
