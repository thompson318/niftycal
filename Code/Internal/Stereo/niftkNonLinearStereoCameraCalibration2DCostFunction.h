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

#include <Internal/niftkNonLinearStereo2DCostFunction.h>
#include <Internal/niftkNonLinearNoIntrinsicsCostFunction.h>

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
  itkTypeMacro(niftk::NonLinearStereoCameraCalibration2DCostFunction, niftk::NonLinearStereo2DCostFunction);

  typedef Superclass::ParametersType                     ParametersType;
  typedef Superclass::DerivativeType                     DerivativeType;
  typedef Superclass::MeasureType                        MeasureType;

  itkSetMacro(OptimiseIntrinsics, bool);
  itkGetConstMacro(OptimiseIntrinsics, bool);

  itkSetMacro(OptimiseExtrinsics, bool);
  itkGetConstMacro(OptimiseExtrinsics, bool);

  itkSetMacro(OptimiseR2L, bool);
  itkGetConstMacro(OptimiseR2L, bool);

  itkSetMacro(Optimise2DOFStereo, bool);
  itkGetConstMacro(Optimise2DOFStereo, bool);

  itkSetMacro(AxisOfRotation, cv::Vec3d);
  itkGetConstMacro(AxisOfRotation, cv::Vec3d);

  itkSetMacro(AngleOfRotation, double);
  itkGetMacro(AngleOfRotation, double);

  itkSetMacro(TranslationVector, cv::Vec3d);
  itkGetConstMacro(TranslationVector, cv::Vec3d);

  void SetExtrinsics(const std::vector<cv::Mat>& rvecsLeft,
                     const std::vector<cv::Mat>& tvecsLeft);

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoCameraCalibration2DCostFunction();
  virtual ~NonLinearStereoCameraCalibration2DCostFunction();

  NonLinearStereoCameraCalibration2DCostFunction(const NonLinearStereoCameraCalibration2DCostFunction&);
  NonLinearStereoCameraCalibration2DCostFunction& operator=(const NonLinearStereoCameraCalibration2DCostFunction&);

private:

  bool                 m_OptimiseIntrinsics;
  bool                 m_OptimiseExtrinsics;
  bool                 m_OptimiseR2L;
  bool                 m_Optimise2DOFStereo;
  cv::Vec3d            m_AxisOfRotation;
  double               m_AngleOfRotation;
  cv::Vec3d            m_TranslationVector;
  std::vector<cv::Mat> m_RVecsLeft;
  std::vector<cv::Mat> m_TVecsLeft;
};

} // end namespace

#endif
