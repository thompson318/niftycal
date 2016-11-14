/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinear12DOFHandEyeCostFunction_h
#define niftkNonLinear12DOFHandEyeCostFunction_h

#include <niftkWin32ExportHeader.h>
#include "niftkNonLinearNoIntrinsicsHandEyeCostFunction.h"

namespace niftk
{

/**
* \class NonLinear12DOFHandEyeCostFunction
* \brief Computes cost as the RMS re-projection error for hand-eye and model-to-world optimisation.
*
* \see niftk::NonLinear12DOFHandEyeOptimiser
*/
class NIFTYCAL_WINEXPORT NonLinear12DOFHandEyeCostFunction :
    public niftk::NonLinearNoIntrinsicsHandEyeCostFunction
{

public:

  typedef NonLinear12DOFHandEyeCostFunction        Self;
  typedef NonLinearNoIntrinsicsHandEyeCostFunction Superclass;
  typedef itk::SmartPointer<Self>                  Pointer;
  typedef itk::SmartPointer<const Self>            ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType               ParametersType;
  typedef Superclass::DerivativeType               DerivativeType;
  typedef Superclass::MeasureType                  MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinear12DOFHandEyeCostFunction();
  virtual ~NonLinear12DOFHandEyeCostFunction();

  NonLinear12DOFHandEyeCostFunction(const NonLinear12DOFHandEyeCostFunction&);
  NonLinear12DOFHandEyeCostFunction& operator=(const NonLinear12DOFHandEyeCostFunction&);

};

} // end namespace

#endif
