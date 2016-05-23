/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMaltiNDOFHandEyeCostFunction_h
#define niftkNonLinearMaltiNDOFHandEyeCostFunction_h

#include <cv.h>
#include "niftkNiftyCalTypes.h"
#include "niftkNonLinearHandEyeCostFunction.h"

namespace niftk
{

/**
* \brief RMS re-projection error for intrinsic, extrinsic, hand-eye and model-to-world optimisation.
*
* Deliberately not exported for external libraries.
*
* \see niftk::NonLinearMaltiNDOFHandEyeOptimiser
*/
class NonLinearMaltiNDOFHandEyeCostFunction : public niftk::NonLinearHandEyeCostFunction
{

public:

  typedef NonLinearMaltiNDOFHandEyeCostFunction Self;
  typedef NonLinearHandEyeCostFunction          Superclass;
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType            ParametersType;
  typedef Superclass::DerivativeType            DerivativeType;
  typedef Superclass::MeasureType               MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;
  void SetIntrinsic(cv::Mat* const intrinsic);
  void SetDistortion(cv::Mat* const distortion);

protected:

  NonLinearMaltiNDOFHandEyeCostFunction();
  virtual ~NonLinearMaltiNDOFHandEyeCostFunction();

  NonLinearMaltiNDOFHandEyeCostFunction(const NonLinearMaltiNDOFHandEyeCostFunction&);
  NonLinearMaltiNDOFHandEyeCostFunction& operator=(const NonLinearMaltiNDOFHandEyeCostFunction&);

private:

  cv::Mat* m_Intrinsic;
  cv::Mat* m_Distortion;
};

} // end namespace

#endif
