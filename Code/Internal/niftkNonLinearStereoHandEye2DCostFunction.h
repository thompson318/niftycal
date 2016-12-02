/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoHandEye2DCostFunction_h
#define niftkNonLinearStereoHandEye2DCostFunction_h

#include "niftkNonLinearStereo2DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoHandEye2DCostFunction
* \brief Computes cost as RMS re-projection error for stereo, extrinsic,
* hand-eye and model-to-world optimisation, as an extension of
* <a href="http://dx.doi.org/10.1002/rcs.1478">Malti 2013</a>.
*/
class NonLinearStereoHandEye2DCostFunction : public NonLinearStereo2DCostFunction
{

public:

  typedef NonLinearStereoHandEye2DCostFunction Self;
  typedef NonLinearStereo2DCostFunction        Superclass;
  typedef itk::SmartPointer<Self>              Pointer;
  typedef itk::SmartPointer<const Self>        ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType           ParametersType;
  typedef Superclass::DerivativeType           DerivativeType;
  typedef Superclass::MeasureType              MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

  void SetLeftIntrinsic(const cv::Mat* const intrinsic);
  void SetLeftDistortion(const cv::Mat* const distortion);
  void SetRightIntrinsic(const cv::Mat* const intrinsic);
  void SetRightDistortion(const cv::Mat* const distortion);

protected:

  NonLinearStereoHandEye2DCostFunction();
  virtual ~NonLinearStereoHandEye2DCostFunction();

  NonLinearStereoHandEye2DCostFunction(const NonLinearStereoHandEye2DCostFunction&);
  NonLinearStereoHandEye2DCostFunction& operator=(const NonLinearStereoHandEye2DCostFunction&);

private:

  cv::Mat *m_LeftIntrinsic;
  cv::Mat *m_LeftDistortion;
  cv::Mat *m_RightIntrinsic;
  cv::Mat *m_RightDistortion;
};

} // end namespace

#endif
