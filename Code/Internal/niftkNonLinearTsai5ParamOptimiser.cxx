/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai5ParamOptimiser.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsai5ParamOptimiser::NonLinearTsai5ParamOptimiser()
{
  m_CostFunction = niftk::NonLinearTsai5ParamCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearTsai5ParamOptimiser::~NonLinearTsai5ParamOptimiser()
{
}


//-----------------------------------------------------------------------------
void NonLinearTsai5ParamOptimiser::SetModel(const Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai5ParamOptimiser::SetPoints(const std::list<PointSet>* const points)
{
  if (points->size() != 1)
  {
    niftkNiftyCalThrow() << "List should only ever be of length: 1";
  }
  m_CostFunction->SetPoints(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai5ParamOptimiser::SetCameraConstants(const double& dxPrime, const cv::Point2d& sensorDimensions, const double& sx)
{
  m_CostFunction->SetCameraConstants(dxPrime, sensorDimensions, sx);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearTsai5ParamOptimiser::Optimise(double& Tz, double& f, double& k1, double& Cx, double& Cy)
{
  niftk::NonLinearTsai5ParamCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(5);

  // Set initial parameters.
  initialParameters[0] = Tz;
  initialParameters[1] = f;
  initialParameters[2] = k1;
  initialParameters[3] = Cx;
  initialParameters[4] = Cy;

  m_CostFunction->SetNumberOfParameters(initialParameters.GetSize());

  // Setup optimiser.
  itk::LevenbergMarquardtOptimizer::Pointer optimiser = itk::LevenbergMarquardtOptimizer::New();
  optimiser->UseCostFunctionGradientOff(); // use default VNL derivative, not our one.
  optimiser->SetCostFunction(m_CostFunction);
  optimiser->SetInitialPosition(initialParameters);
  optimiser->SetNumberOfIterations(1000);
  optimiser->SetGradientTolerance(0.0001);
  optimiser->SetEpsilonFunction(0.0001);
  optimiser->SetValueTolerance(0.0001);

  niftk::NonLinearTsai5ParamCostFunction::MeasureType initialValues = m_CostFunction->GetValue(initialParameters);
  double initialRMS = m_CostFunction->GetRMS(initialValues);

  std::cout << "NonLinearTsai5ParamOptimiser: initial=" << initialParameters
            << ", rms=" << initialRMS << std::endl;

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearTsai5ParamCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  Tz = finalParameters[0];
  f = finalParameters[1];
  k1 = finalParameters[2];
  Cx = finalParameters[3];
  Cy = finalParameters[4];

  niftk::NonLinearTsai5ParamCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  std::cout << "NonLinearTsai5ParamOptimiser: final=" << finalParameters << ", rms=" << finalRMS << std::endl;

  return finalRMS;
}

} // end namespace
