/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearMonoCameraCalibration3DOptimiser.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearMonoCameraCalibration3DOptimiser::NonLinearMonoCameraCalibration3DOptimiser()
{
  m_CostFunction = niftk::NonLinearMonoCameraCalibration3DCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearMonoCameraCalibration3DOptimiser::~NonLinearMonoCameraCalibration3DOptimiser()
{
}


//-----------------------------------------------------------------------------
void NonLinearMonoCameraCalibration3DOptimiser::SetModel(const Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMonoCameraCalibration3DOptimiser::SetPoints(const std::list<PointSet>* const points)
{
  m_CostFunction->SetPoints(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMonoCameraCalibration3DOptimiser::SetIntrinsic(const cv::Mat& intrinsic)
{
  m_CostFunction->SetIntrinsic(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMonoCameraCalibration3DOptimiser::SetDistortion(const cv::Mat& distortion)
{
  m_CostFunction->SetDistortion(distortion);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearMonoCameraCalibration3DOptimiser::Optimise(std::vector<cv::Mat>& rvecs,
                                                           std::vector<cv::Mat>& tvecs
                                                          )
{
  niftk::NonLinearMonoCameraCalibration3DCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(6 * rvecs.size());

  unsigned int parameterCounter = 0;
  for (unsigned int i = 0; i < rvecs.size(); i++)
  {
    initialParameters[parameterCounter++] = rvecs[i].at<double>(0, 0);
    initialParameters[parameterCounter++] = rvecs[i].at<double>(0, 1);
    initialParameters[parameterCounter++] = rvecs[i].at<double>(0, 2);
    initialParameters[parameterCounter++] = tvecs[i].at<double>(0, 0);
    initialParameters[parameterCounter++] = tvecs[i].at<double>(0, 1);
    initialParameters[parameterCounter++] = tvecs[i].at<double>(0, 2);
  }

  m_CostFunction->SetNumberOfParameters(initialParameters.GetSize());

  // Setup optimiser.
  itk::LevenbergMarquardtOptimizer::Pointer optimiser = itk::LevenbergMarquardtOptimizer::New();
  optimiser->UseCostFunctionGradientOff(); // use default VNL derivative, not our one.
  optimiser->SetCostFunction(m_CostFunction);
  optimiser->SetInitialPosition(initialParameters);
  optimiser->SetNumberOfIterations(10000);
  optimiser->SetGradientTolerance(0.0000001);
  optimiser->SetEpsilonFunction(0.0000001);
  optimiser->SetValueTolerance(0.0000001);

  niftk::NonLinearMonoCameraCalibration3DCostFunction::MeasureType initialValues = m_CostFunction->GetValue(initialParameters);
  double initialRMS = m_CostFunction->GetRMS(initialValues);

  std::cout << "NonLinearMonoCameraCalibration3DOptimiser: initialRMS=" << initialRMS << std::endl;

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  parameterCounter = 0;
  niftk::NonLinearMonoCameraCalibration3DCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  for (unsigned int i = 0; i < rvecs.size(); i++)
  {
    rvecs[i].at<double>(0, 0) = finalParameters[parameterCounter++];
    rvecs[i].at<double>(0, 1) = finalParameters[parameterCounter++];
    rvecs[i].at<double>(0, 2) = finalParameters[parameterCounter++];
    tvecs[i].at<double>(0, 0) = finalParameters[parameterCounter++];
    tvecs[i].at<double>(0, 1) = finalParameters[parameterCounter++];
    tvecs[i].at<double>(0, 2) = finalParameters[parameterCounter++];
  }

  niftk::NonLinearMonoCameraCalibration3DCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  for (unsigned int i = 0; i < finalParameters.size(); i++)
  {
    std::cout << "NonLinearMonoCameraCalibration3DOptimiser: " << initialParameters[i]
                 << "\t" << finalParameters[i]
                 << "\t" << finalParameters[i] - initialParameters[i]
                 << std::endl;
  }
  std::cout << "NonLinearMonoCameraCalibration3DOptimiser: finalRMS=" << finalRMS << ", stopped by " << optimiser->GetStopConditionDescription() << std::endl;

  return finalRMS;
}

} // end namespace
