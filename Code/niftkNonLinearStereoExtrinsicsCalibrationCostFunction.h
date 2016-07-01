/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoExtrinsicsCalibrationCostFunction_h
#define niftkNonLinearStereoExtrinsicsCalibrationCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"
#include "niftkNonLinearStereoCalibrationCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoExtrinsicsCalibrationCostFunction
* \brief Computes cost as RMS reconstruction (3D) error
* for extrinsic and stereo optimisation.
*
* \see niftk::NonLinearStereoExtrinsicsCalibrationOptimiser
*/
class NIFTYCAL_WINEXPORT NonLinearStereoExtrinsicsCalibrationCostFunction :
    public niftk::NonLinearStereoCalibrationCostFunction
{

public:

  typedef NonLinearStereoExtrinsicsCalibrationCostFunction Self;
  typedef niftk::NonLinearStereoCalibrationCostFunction    Superclass;
  typedef itk::SmartPointer<Self>                          Pointer;
  typedef itk::SmartPointer<const Self>                    ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType                       ParametersType;
  typedef Superclass::DerivativeType                       DerivativeType;
  typedef Superclass::MeasureType                          MeasureType;

  void SetDistortionParameters(cv::Mat* const leftDistortion,
                               cv::Mat* const rightDistortion
                              );

  void SetIntrinsics(cv::Mat* const leftIntrinsic,
                     cv::Mat* const rightIntrinsic
                    );

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoExtrinsicsCalibrationCostFunction();
  virtual ~NonLinearStereoExtrinsicsCalibrationCostFunction();

  NonLinearStereoExtrinsicsCalibrationCostFunction(const NonLinearStereoExtrinsicsCalibrationCostFunction&);
  NonLinearStereoExtrinsicsCalibrationCostFunction& operator=(const NonLinearStereoExtrinsicsCalibrationCostFunction&);

private:
  cv::Mat             *m_LeftIntrinsic;
  cv::Mat             *m_LeftDistortion;
  cv::Mat             *m_RightIntrinsic;
  cv::Mat             *m_RightDistortion;
};

} // end namespace

#endif
