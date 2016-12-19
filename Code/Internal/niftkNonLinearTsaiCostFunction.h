/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsaiCostFunction_h
#define niftkNonLinearTsaiCostFunction_h

#include "niftkNonLinearMonoCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsaiCostFunction
* \brief Base class for Tsai cost functions.
*/
class NonLinearTsaiCostFunction : public niftk::NonLinearMonoCostFunction
{

public:

  typedef NonLinearTsaiCostFunction     Self;
  typedef NonLinearMonoCostFunction     Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;

  typedef Superclass::ParametersType    ParametersType;
  typedef Superclass::DerivativeType    DerivativeType;
  typedef Superclass::MeasureType       MeasureType;

  void SetExtrinsic(const cv::Matx44d* extrinsic);
  void SetIntrinsic(const cv::Mat* const intrinsic);
  void SetSx(const double& sx);
  void SetK1(const double& k1);

protected:

  NonLinearTsaiCostFunction(); // deliberately protected.
  virtual ~NonLinearTsaiCostFunction(); // deliberately protected.

  NonLinearTsaiCostFunction(const NonLinearTsaiCostFunction&); // deliberately not implemented
  NonLinearTsaiCostFunction& operator=(const NonLinearTsaiCostFunction&); // deliberately not implemented

  cv::Matx44d* m_Extrinsic;
  cv::Mat*     m_Intrinsic;
  double       m_Sx;
  double       m_K1;
};

} // end namespace

#endif
