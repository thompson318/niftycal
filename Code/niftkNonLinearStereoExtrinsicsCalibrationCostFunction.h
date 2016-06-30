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
#include "niftkNonLinearCostFunction.h"

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
    public niftk::NonLinearCostFunction
{

public:

  typedef NonLinearStereoExtrinsicsCalibrationCostFunction Self;
  typedef niftk::NonLinearCostFunction           Superclass;
  typedef itk::SmartPointer<Self>                Pointer;
  typedef itk::SmartPointer<const Self>          ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType             ParametersType;
  typedef Superclass::DerivativeType             DerivativeType;
  typedef Superclass::MeasureType                MeasureType;

  void SetRightHandPoints(std::list<PointSet>* const points);

  void SetIntrinsics(cv::Mat* const leftIntrinsic,
                     cv::Mat* const leftDistortion,
                     cv::Mat* const rightIntrinsic,
                     cv::Mat* const rightDistortion
                    );

  virtual unsigned int GetNumberOfValues(void) const ITK_OVERRIDE;
  itkSetMacro(NumberOfValues, unsigned int);

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoExtrinsicsCalibrationCostFunction();
  virtual ~NonLinearStereoExtrinsicsCalibrationCostFunction();

  NonLinearStereoExtrinsicsCalibrationCostFunction(const NonLinearStereoExtrinsicsCalibrationCostFunction&);
  NonLinearStereoExtrinsicsCalibrationCostFunction& operator=(const NonLinearStereoExtrinsicsCalibrationCostFunction&);

private:
  std::list<PointSet> *m_RightHandPoints;
  cv::Mat             *m_LeftIntrinsic;
  cv::Mat             *m_LeftDistortion;
  cv::Mat             *m_RightIntrinsic;
  cv::Mat             *m_RightDistortion;
};

} // end namespace

#endif
