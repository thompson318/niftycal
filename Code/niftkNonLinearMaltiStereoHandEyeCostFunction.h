/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMaltiStereoHandEyeCostFunction_h
#define niftkNonLinearMaltiStereoHandEyeCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkNonLinearHandEyeCostFunction.h"

namespace niftk
{

/**
* \brief RMS re-projection error for intrinsic, extrinsic, hand-eye and model-to-world optimisation.
*
* \see niftk::NonLinearMaltiStereoHandEyeOptimiser
*/
class NIFTYCAL_WINEXPORT NonLinearMaltiStereoHandEyeCostFunction :
    public niftk::NonLinearHandEyeCostFunction
{

public:

  typedef NonLinearMaltiStereoHandEyeCostFunction Self;
  typedef NonLinearHandEyeCostFunction          Superclass;
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType            ParametersType;
  typedef Superclass::DerivativeType            DerivativeType;
  typedef Superclass::MeasureType               MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;
  void SetLeftIntrinsic(cv::Mat* const intrinsic);
  void SetLeftDistortion(cv::Mat* const distortion);
  void SetRightIntrinsic(cv::Mat* const intrinsic);
  void SetRightDistortion(cv::Mat* const distortion);
  void SetRightHandPoints(std::list<PointSet>* const points);
  virtual unsigned int GetNumberOfValues(void) const ITK_OVERRIDE;

protected:

  NonLinearMaltiStereoHandEyeCostFunction();
  virtual ~NonLinearMaltiStereoHandEyeCostFunction();

  NonLinearMaltiStereoHandEyeCostFunction(const NonLinearMaltiStereoHandEyeCostFunction&);
  NonLinearMaltiStereoHandEyeCostFunction& operator=(const NonLinearMaltiStereoHandEyeCostFunction&);

  void ProjectPoints(const PointSet& points,
                     const cv::Matx44d& extrinsic,
                     const cv::Mat& intrinsic,
                     const cv::Mat& distortion,
                     MeasureType& values,
                     unsigned int& totalPointCounter
                    ) const;

private:

  cv::Mat             *m_LeftIntrinsic;
  cv::Mat             *m_LeftDistortion;
  cv::Mat             *m_RightIntrinsic;
  cv::Mat             *m_RightDistortion;
  std::list<PointSet> *m_RightHandPoints;
  unsigned int         m_NumberOfRightHandValues;
};

} // end namespace

#endif
