/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai5ParamOptimiser_h
#define niftkNonLinearTsai5ParamOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearTsai5ParamCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai5ParamOptimiser
* \brief Optimises Tz, f, k, Cx and Cy using niftk::NonLinearTsai5ParamHandCostFunction.
*/
class NonLinearTsai5ParamOptimiser : public itk::Object
{

public:

  typedef  NonLinearTsai5ParamOptimiser Self;
  typedef  itk::Object                  Superclass;
  typedef  itk::SmartPointer<Self>      Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points); // Must be a list of length 1.
  void SetCameraConstants(const double& dxPrime, const cv::Point2d& sensorDimensions, const double& sx);
  double Optimise(double& Tz, double& f, double& k1, double& Cx, double& Cy);

protected:

  NonLinearTsai5ParamOptimiser(); // deliberately protected.
  virtual ~NonLinearTsai5ParamOptimiser(); // deliberately protected.

  NonLinearTsai5ParamOptimiser(const NonLinearTsai5ParamOptimiser&); // deliberately not implemented
  NonLinearTsai5ParamOptimiser& operator=(const NonLinearTsai5ParamOptimiser&); // deliberately not implemented

private:
  niftk::NonLinearTsai5ParamCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
