/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoCameraCalibration2DCostFunction_h
#define niftkNonLinearStereoCameraCalibration2DCostFunction_h

#include "niftkNonLinearStereo2DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoCameraCalibration2DCostFunction
* \brief Computes cost as RMS re-projection error for intrinsic, extrinsic
* and stereo parameters, optimising projection error.
*/
class NonLinearStereoCameraCalibration2DCostFunction : public NonLinearStereo2DCostFunction
{

public:

  typedef NonLinearStereoCameraCalibration2DCostFunction Self;
  typedef NonLinearStereo2DCostFunction                  Superclass;
  typedef itk::SmartPointer<Self>                        Pointer;
  typedef itk::SmartPointer<const Self>                  ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType                     ParametersType;
  typedef Superclass::DerivativeType                     DerivativeType;
  typedef Superclass::MeasureType                        MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoCameraCalibration2DCostFunction();
  virtual ~NonLinearStereoCameraCalibration2DCostFunction();

  NonLinearStereoCameraCalibration2DCostFunction(const NonLinearStereoCameraCalibration2DCostFunction&);
  NonLinearStereoCameraCalibration2DCostFunction& operator=(const NonLinearStereoCameraCalibration2DCostFunction&);

private:

};

} // end namespace

#endif
