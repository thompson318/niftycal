/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoIntrinsicsCalibrationCostFunction_h
#define niftkNonLinearStereoIntrinsicsCalibrationCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"
#include "niftkNonLinearCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoIntrinsicsCalibrationCostFunction
* \brief Computes cost as RMS reconstruction (3D) error
* where the parameters array contains only intrinsic parameters.
*
* \see niftk::NonLinearStereoIntrinsicsCalibrationOptimiser
*/
class NIFTYCAL_WINEXPORT NonLinearStereoIntrinsicsCalibrationCostFunction :
    public niftk::NonLinearCostFunction
{

public:

  typedef NonLinearStereoIntrinsicsCalibrationCostFunction Self;
  typedef niftk::NonLinearCostFunction                     Superclass;
  typedef itk::SmartPointer<Self>                          Pointer;
  typedef itk::SmartPointer<const Self>                    ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType                       ParametersType;
  typedef Superclass::DerivativeType                       DerivativeType;
  typedef Superclass::MeasureType                          MeasureType;

  void SetRightHandPoints(std::list<PointSet>* const points);

  void SetExtrinsics(std::vector<cv::Mat>* const rvecsLeft,
                     std::vector<cv::Mat>* const tvecsLeft,
                     cv::Mat* const leftToRightRotationMatrix,
                     cv::Mat* const leftToRightTranslationVector
                     );

  void SetDistortionParameters(cv::Mat* const leftDistortion,
                               cv::Mat* const rightDistortion
                               );

  virtual unsigned int GetNumberOfValues(void) const ITK_OVERRIDE;
  itkSetMacro(NumberOfValues, unsigned int);

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoIntrinsicsCalibrationCostFunction();
  virtual ~NonLinearStereoIntrinsicsCalibrationCostFunction();

  NonLinearStereoIntrinsicsCalibrationCostFunction(const NonLinearStereoIntrinsicsCalibrationCostFunction&);
  NonLinearStereoIntrinsicsCalibrationCostFunction& operator=(const NonLinearStereoIntrinsicsCalibrationCostFunction&);

private:
  std::list<PointSet>  *m_RightHandPoints;
  std::vector<cv::Mat> *m_RvecsLeft;
  std::vector<cv::Mat> *m_TvecsLeft;
  cv::Mat              *m_LeftToRightRotationMatrix;
  cv::Mat              *m_LeftToRightTranslationVector;
  cv::Mat              *m_LeftDistortion;
  cv::Mat              *m_RightDistortion;
};

} // end namespace

#endif
