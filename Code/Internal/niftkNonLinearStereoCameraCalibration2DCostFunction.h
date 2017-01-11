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
#include "niftkNonLinearNoIntrinsicsCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoCameraCalibration2DCostFunction
* \brief Computes cost as RMS re-projection error for intrinsic, extrinsic
* and stereo parameters, optimising projection error.
*/
class NonLinearStereoCameraCalibration2DCostFunction
    : public NonLinearStereo2DCostFunction,
      public NonLinearNoIntrinsicsCostFunction
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

  itkSetMacro(OptimiseIntrinsics, bool);
  itkGetConstMacro(OptimiseIntrinsics, bool);

  itkSetMacro(Optimise2DOFStereo, bool);
  itkGetConstMacro(Optimise2DOFStereo, bool);

  itkSetMacro(AxisOfRotation, cv::Vec3d);
  itkGetConstMacro(AxisOfRotation, cv::Vec3d);

  itkSetMacro(TranslationVector, cv::Vec3d);
  itkGetConstMacro(TranslationVector, cv::Vec3d);

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoCameraCalibration2DCostFunction();
  virtual ~NonLinearStereoCameraCalibration2DCostFunction();

  NonLinearStereoCameraCalibration2DCostFunction(const NonLinearStereoCameraCalibration2DCostFunction&);
  NonLinearStereoCameraCalibration2DCostFunction& operator=(const NonLinearStereoCameraCalibration2DCostFunction&);

private:

  bool      m_OptimiseIntrinsics;
  bool      m_Optimise2DOFStereo;
  cv::Vec3d m_AxisOfRotation;
  cv::Vec3d m_TranslationVector;
};

} // end namespace

#endif
