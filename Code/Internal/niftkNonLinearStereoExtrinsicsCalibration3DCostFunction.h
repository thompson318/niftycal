/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoExtrinsicsCalibration3DCostFunction_h
#define niftkNonLinearStereoExtrinsicsCalibration3DCostFunction_h

#include <niftkNiftyCalTypes.h>
#include "niftkNonLinearStereo3DCostFunction.h"
#include "niftkNonLinearNoIntrinsicsCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoExtrinsicsCalibration3DCostFunction
* \brief Computes cost as RMS reconstruction (3D) error
* for extrinsic and stereo optimisation.
*
* \see niftk::NonLinearStereoExtrinsicsCalibration3DOptimiser
*/
class NonLinearStereoExtrinsicsCalibration3DCostFunction :
    public NonLinearStereo3DCostFunction,
    public NonLinearNoIntrinsicsCostFunction
{

public:

  typedef NonLinearStereoExtrinsicsCalibration3DCostFunction Self;
  typedef NonLinearStereo3DCostFunction                      Superclass;
  typedef itk::SmartPointer<Self>                            Pointer;
  typedef itk::SmartPointer<const Self>                      ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType                         ParametersType;
  typedef Superclass::DerivativeType                         DerivativeType;
  typedef Superclass::MeasureType                            MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoExtrinsicsCalibration3DCostFunction();
  virtual ~NonLinearStereoExtrinsicsCalibration3DCostFunction();

  NonLinearStereoExtrinsicsCalibration3DCostFunction(const NonLinearStereoExtrinsicsCalibration3DCostFunction&);
  NonLinearStereoExtrinsicsCalibration3DCostFunction& operator=(const NonLinearStereoExtrinsicsCalibration3DCostFunction&);

};

} // end namespace

#endif
