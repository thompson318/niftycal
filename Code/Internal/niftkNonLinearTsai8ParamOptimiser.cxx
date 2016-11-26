/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai8ParamOptimiser.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsai8ParamOptimiser::NonLinearTsai8ParamOptimiser()
{
  m_CostFunction = niftk::NonLinearTsai8ParamCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearTsai8ParamOptimiser::~NonLinearTsai8ParamOptimiser()
{
}


//-----------------------------------------------------------------------------
void NonLinearTsai8ParamOptimiser::SetModel(const Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai8ParamOptimiser::SetPoints(const std::list<PointSet>* const points)
{
  if (points->size() != 1)
  {
    niftkNiftyCalThrow() << "List should only ever be of length: 1";
  }
  m_CostFunction->SetPoints(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai8ParamOptimiser::SetIntrinsic(const cv::Mat* const intrinsic)
{
  m_CostFunction->SetIntrinsic(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearTsai8ParamOptimiser::Optimise(double& Rx, double& Ry, double& Rz,
                                              double& Tx, double& Ty, double& Tz,
                                              double& f, double& k1)
{
  niftk::NonLinearTsai8ParamCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(8);

  // Set initial parameters.
  initialParameters[0] = Rx;
  initialParameters[1] = Ry;
  initialParameters[2] = Rz;
  initialParameters[3] = Tx;
  initialParameters[4] = Ty;
  initialParameters[5] = Tz;
  initialParameters[6] = f;
  initialParameters[7] = k1;

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

  /*
  niftk::NonLinearTsai8ParamCostFunction::MeasureType initialValues = m_CostFunction->GetValue(initialParameters);
  double initialRMS = m_CostFunction->GetRMS(initialValues);
  std::cout << "NonLinearTsai8ParamOptimiser: initial=" << initialParameters << ", rms=" << initialRMS << std::endl;
  */

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearTsai8ParamCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  Rx = finalParameters[0];
  Ry = finalParameters[1];
  Rz = finalParameters[2];
  Tx = finalParameters[3];
  Ty = finalParameters[4];
  Tz = finalParameters[5];
  f = finalParameters[6];
  k1 = finalParameters[7];

  niftk::NonLinearTsai8ParamCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  //std::cout << "NonLinearTsai8ParamOptimiser: final=" << finalParameters << ", rms=" << finalRMS << std::endl;

  return finalRMS;
}

} // end namespace
