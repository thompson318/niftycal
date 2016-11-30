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

#include "niftkNonLinearMonoCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai5ParamCostFunction
* \brief Cost function, to optimise 2D projection error over Tz, f, k1, Cx and Cy.
* \see niftk::NonLinearTsai5ParamOptimiser
*/
class NonLinearTsai5ParamCostFunction : public niftk::NonLinearMonoCostFunction
{

public:

  typedef NonLinearTsai5ParamCostFunction Self;
  typedef NonLinearMonoCostFunction       Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType      ParametersType;
  typedef Superclass::DerivativeType      DerivativeType;
  typedef Superclass::MeasureType         MeasureType;

  void SetCameraConstants(const double& dxPrime, const cv::Point2d& sensorDimensions, const double& sx);
  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearTsai5ParamCostFunction(); // deliberately protected.
  virtual ~NonLinearTsai5ParamCostFunction(); // deliberately protected.

  NonLinearTsai5ParamCostFunction(const NonLinearTsai5ParamCostFunction&); // deliberately not implemented
  NonLinearTsai5ParamCostFunction& operator=(const NonLinearTsai5ParamCostFunction&); // deliberately not implemented

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
