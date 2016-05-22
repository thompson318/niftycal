/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMalti12DOFHandEyeCostFunction_h
#define niftkNonLinearMalti12DOFHandEyeCostFunction_h

#include <cv.h>
#include "niftkNiftyCalTypes.h"
#include "niftkNonLinearHandEyeCostFunction.h"

namespace niftk
{

/**
* \brief RMS re-projection error for hand-eye and model-to-world optimisation.
*
* Deliberately not exported for external libraries.
*
* \see niftk::NonLinearMalti12DOFHandEyeOptimiser
*/
class NonLinearMalti12DOFHandEyeCostFunction : public niftk::NonLinearHandEyeCostFunction
{

public:

  typedef NonLinearMalti12DOFHandEyeCostFunction Self;
  typedef NonLinearHandEyeCostFunction           Superclass;
  typedef itk::SmartPointer<Self>                Pointer;
  typedef itk::SmartPointer<const Self>          ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType             ParametersType;
  typedef Superclass::DerivativeType             DerivativeType;
  typedef Superclass::MeasureType                MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;
  void SetIntrinsic(cv::Mat* const intrinsic);
  void SetDistortion(cv::Mat* const distortion);

protected:

  NonLinearMalti12DOFHandEyeCostFunction();
  virtual ~NonLinearMalti12DOFHandEyeCostFunction();

  NonLinearMalti12DOFHandEyeCostFunction(const NonLinearMalti12DOFHandEyeCostFunction&);
  NonLinearMalti12DOFHandEyeCostFunction& operator=(const NonLinearMalti12DOFHandEyeCostFunction&);

private:

  cv::Mat* m_Intrinsic;
  cv::Mat* m_Distortion;
};

} // end namespace

#endif
