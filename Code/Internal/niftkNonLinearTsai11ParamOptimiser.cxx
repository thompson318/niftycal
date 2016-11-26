/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsai11ParamOptimiser.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsai11ParamOptimiser::NonLinearTsai11ParamOptimiser()
{
  m_CostFunction = niftk::NonLinearTsai11ParamCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearTsai11ParamOptimiser::~NonLinearTsai11ParamOptimiser()
{
}


//-----------------------------------------------------------------------------
void NonLinearTsai11ParamOptimiser::SetModel(const Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearTsai11ParamOptimiser::SetPoints(const std::list<PointSet>* const points)
{
  if (points->size() != 1)
  {
    niftkNiftyCalThrow() << "List should only ever be of length: 1";
  }
  m_CostFunction->SetPoints(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearTsai11ParamOptimiser::Optimise(double& Rx, double& Ry, double& Rz,
                                               double& Tx, double& Ty, double& Tz,
                                               double& fx, double& fy,
                                               double& Cx, double& Cy,
                                               double& k1
                                              )
{
  niftk::NonLinearTsai11ParamCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(11);

  // Set initial parameters.
  initialParameters[0] = Rx;
  initialParameters[1] = Ry;
  initialParameters[2] = Rz;
  initialParameters[3] = Tx;
  initialParameters[4] = Ty;
  initialParameters[5] = Tz;
  initialParameters[6] = fx;
  initialParameters[7] = fy;
  initialParameters[8] = Cx;
  initialParameters[9] = Cy;
  initialParameters[10] = k1;

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
  niftk::NonLinearTsai11ParamCostFunction::MeasureType initialValues = m_CostFunction->GetValue(initialParameters);
  double initialRMS = m_CostFunction->GetRMS(initialValues);
  std::cout << "NonLinearTsai11ParamOptimiser: initial=" << initialParameters << ", rms=" << initialRMS << std::endl;
  */

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearTsai11ParamCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  Rx = finalParameters[0];
  Ry = finalParameters[1];
  Rz = finalParameters[2];
  Tx = finalParameters[3];
  Ty = finalParameters[4];
  Tz = finalParameters[5];
  fx = finalParameters[6];
  fy = finalParameters[7];
  Cx = finalParameters[8];
  Cy = finalParameters[9];
  k1 = finalParameters[10];

  niftk::NonLinearTsai11ParamCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  //std::cout << "NonLinearTsai11ParamOptimiser: final=" << finalParameters << ", rms=" << finalRMS << std::endl;

  return finalRMS;
}

} // end namespace
