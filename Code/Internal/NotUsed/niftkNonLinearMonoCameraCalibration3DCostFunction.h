/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMonoCameraCalibration3DCostFunction_h
#define niftkNonLinearMonoCameraCalibration3DCostFunction_h

#include "niftkNonLinearMonoCostFunction3D.h"

namespace niftk
{

/**
* \class NonLinearMonoCameraCalibration3DCostFunction
* \brief Computes cost as RMS reconstruction error for intrinsic, distortion and extrinsic params.
* \see niftk::NonLinearMonoCameraCalibration3DOptimiser
*/
class NonLinearMonoCameraCalibration3DCostFunction : public NonLinearMonoCostFunction3D
{

public:

  typedef NonLinearMonoCameraCalibration3DCostFunction Self;
  typedef NonLinearMonoCostFunction3D                  Superclass;
  typedef itk::SmartPointer<Self>                      Pointer;
  typedef itk::SmartPointer<const Self>                ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType                   ParametersType;
  typedef Superclass::DerivativeType                   DerivativeType;
  typedef Superclass::MeasureType                      MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;
  void SetIntrinsic(const cv::Mat& intrinsic);
  void SetDistortion(const cv::Mat& distortion);

protected:

  NonLinearMonoCameraCalibration3DCostFunction();
  virtual ~NonLinearMonoCameraCalibration3DCostFunction();

  NonLinearMonoCameraCalibration3DCostFunction(const NonLinearMonoCameraCalibration3DCostFunction&);
  NonLinearMonoCameraCalibration3DCostFunction& operator=(const NonLinearMonoCameraCalibration3DCostFunction&);

  cv::Mat m_Intrinsic;
  cv::Mat m_Distortion;
};

} // end namespace

#endif
