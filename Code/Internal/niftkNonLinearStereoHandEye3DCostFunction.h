/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoHandEye3DCostFunction_h
#define niftkNonLinearStereoHandEye3DCostFunction_h

#include "niftkNonLinearStereo2DCostFunction.h"
#include "niftkNonLinearNoIntrinsicsCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoHandEye3DCostFunction
* \brief Computes cost as RMS re-projection error for stereo, extrinsic,
* hand-eye and model-to-world optimisation.
*/
class NonLinearStereoHandEye3DCostFunction
    : public NonLinearStereo2DCostFunction,
      public NonLinearNoIntrinsicsCostFunction
{

public:

  typedef NonLinearStereoHandEye3DCostFunction Self;
  typedef NonLinearStereo2DCostFunction        Superclass;
  typedef itk::SmartPointer<Self>              Pointer;
  typedef itk::SmartPointer<const Self>        ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType           ParametersType;
  typedef Superclass::DerivativeType           DerivativeType;
  typedef Superclass::MeasureType              MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoHandEye3DCostFunction();
  virtual ~NonLinearStereoHandEye3DCostFunction();

  NonLinearStereoHandEye3DCostFunction(const NonLinearStereoHandEye3DCostFunction&);
  NonLinearStereoHandEye3DCostFunction& operator=(const NonLinearStereoHandEye3DCostFunction&);

};

} // end namespace

#endif
