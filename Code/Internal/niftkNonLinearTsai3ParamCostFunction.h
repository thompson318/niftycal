/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai3ParamCostFunction_h
#define niftkNonLinearTsai3ParamCostFunction_h

#include "niftkNonLinearTsaiCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai3ParamCostFunction
* \brief Cost function, to optimise 2D reprojection error over Tz, f and k1.
* \see niftk::NonLinearTsai3ParamOptimiser
*/
class NonLinearTsai3ParamCostFunction : public niftk::NonLinearTsaiCostFunction
{

public:

  typedef NonLinearTsai3ParamCostFunction Self;
  typedef NonLinearTsaiCostFunction       Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType      ParametersType;
  typedef Superclass::DerivativeType      DerivativeType;
  typedef Superclass::MeasureType         MeasureType;

  void SetExtrinsic(const cv::Matx44d* extrinsic);
  void SetIntrinsic(const cv::Mat* const intrinsic);

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearTsai3ParamCostFunction(); // deliberately protected.
  virtual ~NonLinearTsai3ParamCostFunction(); // deliberately protected.

  NonLinearTsai3ParamCostFunction(const NonLinearTsai3ParamCostFunction&); // deliberately not implemented
  NonLinearTsai3ParamCostFunction& operator=(const NonLinearTsai3ParamCostFunction&); // deliberately not implemented

  cv::Matx44d* m_Extrinsic;
  cv::Mat*     m_Intrinsic;
};

} // end namespace

#endif
