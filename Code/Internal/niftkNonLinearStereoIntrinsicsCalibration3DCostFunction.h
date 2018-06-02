/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoIntrinsicsCalibration3DCostFunction_h
#define niftkNonLinearStereoIntrinsicsCalibration3DCostFunction_h

#include <niftkNiftyCalTypes.h>
#include "niftkNonLinearStereo3DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoIntrinsicsCalibration3DCostFunction
* \brief Computes cost as RMS reconstruction (3D) error
* where the parameters array contains only intrinsic parameters.
*
* \see niftk::NonLinearStereoIntrinsicsCalibration3DOptimiser
*/
class NonLinearStereoIntrinsicsCalibration3DCostFunction :
    public NonLinearStereo3DCostFunction
{

public:

  typedef NonLinearStereoIntrinsicsCalibration3DCostFunction Self;
  typedef NonLinearStereo3DCostFunction                      Superclass;
  typedef itk::SmartPointer<Self>                            Pointer;
  typedef itk::SmartPointer<const Self>                      ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType                         ParametersType;
  typedef Superclass::DerivativeType                         DerivativeType;
  typedef Superclass::MeasureType                            MeasureType;

  void SetDistortionParameters(cv::Mat* const leftDistortion,
                               cv::Mat* const rightDistortion
                               );

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoIntrinsicsCalibration3DCostFunction();
  virtual ~NonLinearStereoIntrinsicsCalibration3DCostFunction();

  NonLinearStereoIntrinsicsCalibration3DCostFunction(const NonLinearStereoIntrinsicsCalibration3DCostFunction&);
  NonLinearStereoIntrinsicsCalibration3DCostFunction& operator=(const NonLinearStereoIntrinsicsCalibration3DCostFunction&);

private:
  cv::Mat *m_LeftDistortion;
  cv::Mat *m_RightDistortion;
};

} // end namespace

#endif
