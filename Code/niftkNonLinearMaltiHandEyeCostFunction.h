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

#include <cv.h>
#include "niftkNiftyCalTypes.h"
#include "niftkNonLinearHandEyeCostFunction.h"

namespace niftk
{

/**
* \brief RMS re-projection error for intrinsic, hand-eye and model-to-world
* optimisation as per Malti 2013 paper.
*
* Cost function for non-linear optimisation as per Malti 2013
* paper <a href="http://dx.doi.org/10.1002/rcs.1478">
* Hand-eye and radial distortion calibration for rigid endoscopes</a>.
*
* Deliberately not exported for external libraries.
*
* \see niftk::NonLinearMaltiHandEyeOptimiser
*/
class NonLinearMaltiHandEyeCostFunction : public niftk::NonLinearHandEyeCostFunction
{

public:

  typedef NonLinearMaltiHandEyeCostFunction     Self;
  typedef NonLinearHandEyeCostFunction          Superclass;
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType            ParametersType;
  typedef Superclass::DerivativeType            DerivativeType;
  typedef Superclass::MeasureType               MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearMaltiHandEyeCostFunction();
  virtual ~NonLinearMaltiHandEyeCostFunction();

  NonLinearMaltiHandEyeCostFunction(const NonLinearMaltiHandEyeCostFunction&);
  NonLinearMaltiHandEyeCostFunction& operator=(const NonLinearMaltiHandEyeCostFunction&);

private:

};

} // end namespace

#endif
