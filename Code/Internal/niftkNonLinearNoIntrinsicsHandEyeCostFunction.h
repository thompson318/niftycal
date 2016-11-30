/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearNoIntrinsicsHandEyeCostFunction_h
#define niftkNonLinearNoIntrinsicsHandEyeCostFunction_h

#include "niftkNonLinearMonoCostFunction.h"

namespace niftk
{

/**
* \class NonLinearNoIntrinsicsCostFunction
* \brief Base class for hand-eye cost functions that don't optimise intrinsic parameters.
*/
class NonLinearNoIntrinsicsHandEyeCostFunction :
    public niftk::NonLinearMonoCostFunction
{

public:

  typedef NonLinearNoIntrinsicsHandEyeCostFunction Self;
  typedef NonLinearMonoCostFunction                Superclass;
  typedef itk::SmartPointer<Self>                  Pointer;
  typedef itk::SmartPointer<const Self>            ConstPointer;

  typedef Superclass::ParametersType               ParametersType;
  typedef Superclass::DerivativeType               DerivativeType;
  typedef Superclass::MeasureType                  MeasureType;

  void SetIntrinsic(const cv::Mat* const intrinsic);
  void SetDistortion(const cv::Mat* const distortion);

protected:

  NonLinearNoIntrinsicsHandEyeCostFunction();
  virtual ~NonLinearNoIntrinsicsHandEyeCostFunction();

  NonLinearNoIntrinsicsHandEyeCostFunction(const NonLinearNoIntrinsicsHandEyeCostFunction&);
  NonLinearNoIntrinsicsHandEyeCostFunction& operator=(const NonLinearNoIntrinsicsHandEyeCostFunction&);

  cv::Mat* m_Intrinsic;
  cv::Mat* m_Distortion;

};

} // end namespace

#endif
