/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsaiNormalisingCostFunction_h
#define niftkNonLinearTsaiNormalisingCostFunction_h

#include "niftkNonLinearTsaiCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsaiNormalisingCostFunction
* \brief Base class for Tsai cost functions that normalise points w.r.t. image centre.
*/
class NonLinearTsaiNormalisingCostFunction : public niftk::NonLinearTsaiCostFunction
{

public:

  typedef NonLinearTsaiNormalisingCostFunction Self;
  typedef NonLinearTsaiCostFunction            Superclass;
  typedef itk::SmartPointer<Self>              Pointer;
  typedef itk::SmartPointer<const Self>        ConstPointer;

  typedef Superclass::ParametersType           ParametersType;
  typedef Superclass::DerivativeType           DerivativeType;
  typedef Superclass::MeasureType              MeasureType;

  void SetCameraConstants(const double& dxPrime, const cv::Point2d& sensorDimensions, const double& sx);

protected:

  NonLinearTsaiNormalisingCostFunction(); // deliberately protected.
  virtual ~NonLinearTsaiNormalisingCostFunction(); // deliberately protected.

  NonLinearTsaiNormalisingCostFunction(const NonLinearTsaiNormalisingCostFunction&); // deliberately not implemented
  NonLinearTsaiNormalisingCostFunction& operator=(const NonLinearTsaiNormalisingCostFunction&); // deliberately not implemented

  void ComputeRTxAndTy(const niftk::Model3D& model,
                       const niftk::PointSet& points,
                       const cv::Point2d& imageCentre,
                       cv::Mat& rvec,
                       double& Tx,
                       double& Ty
                      ) const;

  double       m_DxPrime;
  cv::Point2d  m_SensorDimensions;
  double       m_Sx;

};

} // end namespace

#endif
