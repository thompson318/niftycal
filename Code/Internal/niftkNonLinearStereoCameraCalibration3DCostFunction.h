/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoCameraCalibration3DCostFunction_h
#define niftkNonLinearStereoCameraCalibration3DCostFunction_h

#include <niftkNiftyCalTypes.h>
#include "niftkNonLinearStereo3DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoCameraCalibration3DCostFunction
* \brief Computes cost as RMS reconstruction (3D) error
* for intrinsic, extrinsic and stereo optimisation.
*
* \see niftk::NonLinearStereoCameraCalibration3DOptimiser
*/
class NonLinearStereoCameraCalibration3DCostFunction :
    public NonLinearStereo3DCostFunction
{

public:

  typedef NonLinearStereoCameraCalibration3DCostFunction Self;
  typedef NonLinearStereo3DCostFunction                  Superclass;
  typedef itk::SmartPointer<Self>                        Pointer;
  typedef itk::SmartPointer<const Self>                  ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType                     ParametersType;
  typedef Superclass::DerivativeType                     DerivativeType;
  typedef Superclass::MeasureType                        MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoCameraCalibration3DCostFunction();
  virtual ~NonLinearStereoCameraCalibration3DCostFunction();

  NonLinearStereoCameraCalibration3DCostFunction(const NonLinearStereoCameraCalibration3DCostFunction&);
  NonLinearStereoCameraCalibration3DCostFunction& operator=(const NonLinearStereoCameraCalibration3DCostFunction&);
};

} // end namespace

#endif
