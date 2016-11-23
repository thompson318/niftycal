/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai5ParamCostFunction_h
#define niftkNonLinearTsai5ParamCostFunction_h

#include "niftkNonLinearCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai5ParamCostFunction
* \brief Cost function, to optimise 2D reprojection error over Tz, f, k1, Cx and Cy.
* \see niftk::NonLinearTsai5ParamOptimiser
*/
class NonLinearTsai5ParamCostFunction : public niftk::NonLinearCostFunction
{

public:

  typedef NonLinearTsai5ParamCostFunction Self;
  typedef NonLinearCostFunction           Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType      ParametersType;
  typedef Superclass::DerivativeType      DerivativeType;
  typedef Superclass::MeasureType         MeasureType;

  void SetExtrinsic(const cv::Matx44d* extrinsic);
  void SetIntrinsic(const cv::Mat* const intrinsic);
  void SetDistortion(const cv::Mat* const distortion);
  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearTsai5ParamCostFunction(); // deliberately protected.
  virtual ~NonLinearTsai5ParamCostFunction(); // deliberately protected.

  NonLinearTsai5ParamCostFunction(const NonLinearTsai5ParamCostFunction&); // deliberately not implemented
  NonLinearTsai5ParamCostFunction& operator=(const NonLinearTsai5ParamCostFunction&); // deliberately not implemented

  cv::Matx44d* m_Extrinsic;
  cv::Mat*     m_Intrinsic;
  cv::Mat*     m_Distortion;
};

} // end namespace

#endif
