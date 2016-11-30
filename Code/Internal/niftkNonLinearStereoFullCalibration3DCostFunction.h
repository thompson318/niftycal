/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoFullCalibration3DCostFunction_h
#define niftkNonLinearStereoFullCalibration3DCostFunction_h

#include <niftkNiftyCalTypes.h>
#include "niftkNonLinearStereo3DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoFullCalibration3DCostFunction
* \brief Computes cost as RMS reconstruction (3D) error
* for intrinsic, extrinsic and stereo optimisation.
*
* \see niftk::NonLinearStereoFullCalibration3DOptimiser
*/
class NonLinearStereoFullCalibration3DCostFunction :
    public NonLinearStereo3DCostFunction
{

public:

  typedef NonLinearStereoFullCalibration3DCostFunction Self;
  typedef NonLinearStereo3DCostFunction                Superclass;
  typedef itk::SmartPointer<Self>                      Pointer;
  typedef itk::SmartPointer<const Self>                ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType                   ParametersType;
  typedef Superclass::DerivativeType                   DerivativeType;
  typedef Superclass::MeasureType                      MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoFullCalibration3DCostFunction();
  virtual ~NonLinearStereoFullCalibration3DCostFunction();

  NonLinearStereoFullCalibration3DCostFunction(const NonLinearStereoFullCalibration3DCostFunction&);
  NonLinearStereoFullCalibration3DCostFunction& operator=(const NonLinearStereoFullCalibration3DCostFunction&);
};

} // end namespace

#endif
