/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearHandEyeCostFunction_h
#define niftkNonLinearHandEyeCostFunction_h

#include <cv.h>
#include <itkMultipleValuedCostFunction.h>

namespace niftk
{

/**
* \brief RMS re-projection error for intrinsic, hand-eye, model-to-world
* as per Malti 2013 paper.
*
* Cost function for non-linear optimisation as per Malti 2013
* paper "Hand-eye and radial distortion calibration for rigid endoscopes",
* http://dx.doi.org/10.1002/rcs.1478.
*
* Deliberately not exported for external libraries.
*
* \see niftk::NonLinearHandEyeOptimiser
*/
class NonLinearHandEyeCostFunction : public itk::MultipleValuedCostFunction
{

public:

  typedef NonLinearHandEyeCostFunction          Self;
  typedef itk::MultipleValuedCostFunction       Superclass;
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;

  typedef Superclass::ParametersType            ParametersType;
  typedef Superclass::DerivativeType            DerivativeType;
  typedef Superclass::MeasureType               MeasureType;

  virtual unsigned int GetNumberOfValues(void) const override;

  virtual unsigned int GetNumberOfParameters() const override;

  virtual void GetDerivative( const ParametersType & parameters, DerivativeType  & derivative ) const override;

  virtual MeasureType GetValue( const ParametersType & parameters ) const override;

protected:

  NonLinearHandEyeCostFunction();
  virtual ~NonLinearHandEyeCostFunction();

  NonLinearHandEyeCostFunction(const NonLinearHandEyeCostFunction&); // Purposefully not implemented.
  NonLinearHandEyeCostFunction& operator=(const NonLinearHandEyeCostFunction&); // Purposefully not implemented.

};

} // end namespace

#endif
