/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoIntrinsicsCalibration3DOptimiser.h"
#include "niftkNonLinearStereoIntrinsicsCalibration3DCostFunction.h"
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibration3DOptimiser::NonLinearStereoIntrinsicsCalibration3DOptimiser()
{
  m_CostFunction = niftk::NonLinearStereoIntrinsicsCalibration3DCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibration3DOptimiser::~NonLinearStereoIntrinsicsCalibration3DOptimiser()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibration3DOptimiser::SetModelAndPoints(const Model3D* const model,
                                                                        const std::list<PointSet>* const leftPoints,
                                                                        const std::list<PointSet>* const rightPoints
                                                                       )
{
  m_CostFunction->SetModel(const_cast<Model3D* const>(model));
  m_CostFunction->SetPoints(const_cast<std::list<PointSet>* const>(leftPoints));
  m_CostFunction->SetRightHandPoints(const_cast<std::list<PointSet>* const>(rightPoints));
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibration3DOptimiser::SetExtrinsics(std::vector<cv::Mat>* const rvecsLeft,
                                                                    std::vector<cv::Mat>* const tvecsLeft,
                                                                    cv::Mat* const leftToRightRotationMatrix,
                                                                    cv::Mat* const leftToRightTranslationVector
                                                                   )
{
  m_CostFunction->SetExtrinsics(rvecsLeft,
                                tvecsLeft,
                                leftToRightRotationMatrix,
                                leftToRightTranslationVector
                               );
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibration3DOptimiser::SetDistortionParameters(cv::Mat* const leftDistortion,
                                                                              cv::Mat* const rightDistortion
                                                                             )
{
  m_CostFunction->SetDistortionParameters(leftDistortion,
                                          rightDistortion
                                         );
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearStereoIntrinsicsCalibration3DOptimiser::Optimise(cv::Mat& leftIntrinsic,
                                                               cv::Mat& rightIntrinsic
                                                              )
{
  if (leftIntrinsic.rows != 3 || leftIntrinsic.cols != 3)
  {
    niftkNiftyCalThrow() << "Left intrinsic matrix should be 3x3, and its ("
                         << leftIntrinsic.cols << ", " << leftIntrinsic.rows << ")";
  }

  if (rightIntrinsic.rows != 3 || rightIntrinsic.cols != 3)
  {
    niftkNiftyCalThrow() << "Right intrinsic matrix should be 3x3, and its ("
                         << rightIntrinsic.cols << ", " << rightIntrinsic.rows << ")";
  }

  niftk::NonLinearStereoIntrinsicsCalibration3DCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(  4  // left intrinsic
                            + 4  // right intrinsic
                           );

  // Set initial parameters.
  int counter = 0;
  initialParameters[counter++] = leftIntrinsic.at<double>(0, 0);
  initialParameters[counter++] = leftIntrinsic.at<double>(1, 1);
  initialParameters[counter++] = leftIntrinsic.at<double>(0, 2);
  initialParameters[counter++] = leftIntrinsic.at<double>(1, 2);
  initialParameters[counter++] = rightIntrinsic.at<double>(0, 0);
  initialParameters[counter++] = rightIntrinsic.at<double>(1, 1);
  initialParameters[counter++] = rightIntrinsic.at<double>(0, 2);
  initialParameters[counter++] = rightIntrinsic.at<double>(1, 2);

  m_CostFunction->SetNumberOfParameters(initialParameters.GetSize());

  // Setup optimiser.
  itk::LevenbergMarquardtOptimizer::Pointer optimiser = itk::LevenbergMarquardtOptimizer::New();
  optimiser->UseCostFunctionGradientOff(); // use default VNL derivative, not our one.
  optimiser->SetCostFunction(m_CostFunction);
  optimiser->SetInitialPosition(initialParameters);
  optimiser->SetNumberOfIterations(100);
  optimiser->SetGradientTolerance(0.0001);
  optimiser->SetEpsilonFunction(0.0001);
  optimiser->SetValueTolerance(0.001);

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearStereoIntrinsicsCalibration3DCostFunction::ParametersType finalParameters
      = optimiser->GetCurrentPosition();
  niftk::NonLinearStereoIntrinsicsCalibration3DCostFunction::MeasureType finalValues
      = m_CostFunction->GetValue(finalParameters);

  double finalRMS = m_CostFunction->GetRMS(finalValues);

  counter = 0;
  leftIntrinsic.at<double>(0, 0) = finalParameters[counter++];
  leftIntrinsic.at<double>(1, 1) = finalParameters[counter++];
  leftIntrinsic.at<double>(0, 2) = finalParameters[counter++];
  leftIntrinsic.at<double>(1, 2) = finalParameters[counter++];
  rightIntrinsic.at<double>(0, 0) = finalParameters[counter++];
  rightIntrinsic.at<double>(1, 1) = finalParameters[counter++];
  rightIntrinsic.at<double>(0, 2) = finalParameters[counter++];
  rightIntrinsic.at<double>(1, 2) = finalParameters[counter++];

  return finalRMS;
}

} // end namespace
