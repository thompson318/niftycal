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

#include "niftkNonLinearCostFunction.h"
#include <cv.h>

namespace niftk
{

/**
* \class NonLinearTsaiCostFunction
* \brief Base class for Tsai cost functions.
*/
class NonLinearTsaiCostFunction : public niftk::NonLinearCostFunction
{

public:

  typedef NonLinearTsaiCostFunction     Self;
  typedef NonLinearCostFunction         Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;

  typedef Superclass::ParametersType    ParametersType;
  typedef Superclass::DerivativeType    DerivativeType;
  typedef Superclass::MeasureType       MeasureType;

protected:

  NonLinearTsaiCostFunction(); // deliberately protected.
  virtual ~NonLinearTsaiCostFunction(); // deliberately protected.

  NonLinearTsaiCostFunction(const NonLinearTsaiCostFunction&); // deliberately not implemented
  NonLinearTsaiCostFunction& operator=(const NonLinearTsaiCostFunction&); // deliberately not implemented

  void ComputeErrorValues(const niftk::Model3D& model,
                          const niftk::PointSet& points,
                          const cv::Matx44d& extrinsic,
                          const cv::Mat& intrinsic,
                          const cv::Mat& distortion,
                          MeasureType& errorValues
                         ) const;

};

} // end namespace

#endif
