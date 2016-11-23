/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearNDOFHandEyeCostFunction_h
#define niftkNonLinearNDOFHandEyeCostFunction_h

#include "niftkNonLinearNoIntrinsicsHandEyeCostFunction.h"

namespace niftk
{

/**
* \class NonLinearNDOFHandEyeCostFunction
* \brief Computes cost as RMS re-projection error for extrinsic,
* hand-eye and model-to-world optimisation, as an extension of
* <a href="http://dx.doi.org/10.1002/rcs.1478">Malti 2013</a>.
*/
class NonLinearNDOFHandEyeCostFunction :
    public niftk::NonLinearNoIntrinsicsHandEyeCostFunction
{

public:

  typedef NonLinearNDOFHandEyeCostFunction         Self;
  typedef NonLinearNoIntrinsicsHandEyeCostFunction Superclass;
  typedef itk::SmartPointer<Self>                  Pointer;
  typedef itk::SmartPointer<const Self>            ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType               ParametersType;
  typedef Superclass::DerivativeType               DerivativeType;
  typedef Superclass::MeasureType                  MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearNDOFHandEyeCostFunction();
  virtual ~NonLinearNDOFHandEyeCostFunction();

  NonLinearNDOFHandEyeCostFunction(const NonLinearNDOFHandEyeCostFunction&);
  NonLinearNDOFHandEyeCostFunction& operator=(const NonLinearNDOFHandEyeCostFunction&);
};

} // end namespace

#endif
