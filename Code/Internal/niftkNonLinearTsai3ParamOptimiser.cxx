/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai3ParamOptimiser.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsai3ParamOptimiser::NonLinearTsai3ParamOptimiser()
{
  m_CostFunction = niftk::NonLinearTsai3ParamCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearTsai3ParamOptimiser::~NonLinearTsai3ParamOptimiser()
{
}


//-----------------------------------------------------------------------------
void NonLinearTsai3ParamOptimiser::SetModel(const Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai3ParamOptimiser::SetPoints(const std::list<PointSet>* const points)
{
  if (points->size() != 1)
  {
    niftkNiftyCalThrow() << "List should only ever be of length: 1";
  }
  m_CostFunction->SetPoints(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai3ParamOptimiser::SetExtrinsic(const cv::Matx44d* extrinsic)
{
  m_CostFunction->SetExtrinsic(extrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai3ParamOptimiser::SetIntrinsic(const cv::Mat* const intrinsic)
{
  m_CostFunction->SetIntrinsic(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai3ParamOptimiser::SetDistortion(const cv::Mat* const distortion)
{
  m_CostFunction->SetDistortion(distortion);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearTsai3ParamOptimiser::Optimise(double& Tz, double& f, double& k1)
{
  niftk::NonLinearTsai3ParamCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(3);

  // Set initial parameters.
  initialParameters[0] = Tz;
  initialParameters[1] = f;
  initialParameters[2] = k1;

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

  niftk::NonLinearTsai3ParamCostFunction::MeasureType initialValues = m_CostFunction->GetValue(initialParameters);
  double initialRMS = m_CostFunction->GetRMS(initialValues);

  std::cout << "NonLinearTsai3ParamOptimiser: initial=" << initialParameters
            << ", rms=" << initialRMS << std::endl;

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearTsai3ParamCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  Tz = finalParameters[0];
  f = finalParameters[1];
  k1 = finalParameters[2];

  niftk::NonLinearTsai3ParamCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  std::cout << "NonLinearTsai3ParamOptimiser: final=" << finalParameters << ", rms=" << finalRMS << std::endl;

  return finalRMS;
}

} // end namespace
