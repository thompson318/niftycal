/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoExtrinsicsCalibration3DOptimiser.h"
#include "niftkNonLinearStereoExtrinsicsCalibration3DCostFunction.h"
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibration3DOptimiser::NonLinearStereoExtrinsicsCalibration3DOptimiser()
{
  m_CostFunction = niftk::NonLinearStereoExtrinsicsCalibration3DCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibration3DOptimiser::~NonLinearStereoExtrinsicsCalibration3DOptimiser()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibration3DOptimiser::SetModelAndPoints(const Model3D* const model,
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
void NonLinearStereoExtrinsicsCalibration3DOptimiser::SetDistortionParameters(cv::Mat* const leftDistortion,
                                                                              cv::Mat* const rightDistortion
                                                                             )
{
  m_CostFunction->SetDistortion(leftDistortion);
  m_CostFunction->SetRightDistortion(rightDistortion);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibration3DOptimiser::SetIntrinsics(cv::Mat* const leftIntrinsic,
                                                                    cv::Mat* const rightIntrinsic
                                                                   )
{
  m_CostFunction->SetIntrinsic(leftIntrinsic);
  m_CostFunction->SetRightIntrinsic(rightIntrinsic);
  this->Modified();
}


//----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibration3DOptimiser::SetOptimiseCameraExtrinsics(const bool& optimise)
{
  m_CostFunction->SetOptimiseCameraExtrinsics(optimise);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibration3DOptimiser::SetOptimiseL2R(const bool& optimise)
{
  m_CostFunction->SetOptimiseL2R(optimise);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearStereoExtrinsicsCalibration3DOptimiser::Optimise(std::vector<cv::Mat>& rvecsLeft,
                                                                 std::vector<cv::Mat>& tvecsLeft,
                                                                 cv::Mat& leftToRightRotationMatrix,
                                                                 cv::Mat& leftToRightTranslationVector
                                                                )
{
  if (leftToRightRotationMatrix.rows != 3 || leftToRightRotationMatrix.cols != 3)
  {
    niftkNiftyCalThrow() << "Left to Right rotation matrix should be 3x3, and its ("
                         << leftToRightRotationMatrix.cols << ", " << leftToRightRotationMatrix.rows << ")";
  }

  if (leftToRightTranslationVector.rows != 3 || leftToRightTranslationVector.cols != 1)
  {
    niftkNiftyCalThrow() << "Left to Right translation vector matrix should be 3x1, and its ("
                         << leftToRightTranslationVector.rows << ", " << leftToRightTranslationVector.cols << ")";
  }

  if (rvecsLeft.size() != tvecsLeft.size())
  {
    niftkNiftyCalThrow() << "Unequal extrinsic vectors: " << rvecsLeft.size()
                         << ", versus " << tvecsLeft.size();
  }

  cv::Mat leftToRightRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);

  unsigned int numberOfParameters = 0;
  if (this->m_CostFunction->GetOptimiseL2R())
  {
    numberOfParameters += 6;
  }
  if (this->m_CostFunction->GetOptimiseCameraExtrinsics())
  {
    numberOfParameters += (rvecsLeft.size() * 6);
  }

  niftk::NonLinearStereoExtrinsicsCalibration3DCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(numberOfParameters);

  // Set initial parameters.
  int counter = 0;
  if (m_CostFunction->GetOptimiseL2R())
  {
    initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 0);
    initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 1);
    initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 2);
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(0, 0);
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(1, 0);
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(2, 0);
  }
  if (m_CostFunction->GetOptimiseCameraExtrinsics())
  {
    for (int i = 0; i < rvecsLeft.size(); i++)
    {
      initialParameters[counter++] = rvecsLeft[i].at<double>(0, 0);
      initialParameters[counter++] = rvecsLeft[i].at<double>(0, 1);
      initialParameters[counter++] = rvecsLeft[i].at<double>(0, 2);
      initialParameters[counter++] = tvecsLeft[i].at<double>(0, 0);
      initialParameters[counter++] = tvecsLeft[i].at<double>(0, 1);
      initialParameters[counter++] = tvecsLeft[i].at<double>(0, 2);
    }
  }

  m_CostFunction->SetNumberOfParameters(initialParameters.GetSize());
  m_CostFunction->SetExtrinsics(&rvecsLeft, &tvecsLeft, &leftToRightRotationMatrix, &leftToRightTranslationVector);

  // Setup optimiser.
  itk::LevenbergMarquardtOptimizer::Pointer optimiser = itk::LevenbergMarquardtOptimizer::New();
  optimiser->UseCostFunctionGradientOff(); // use default VNL derivative, not our one.
  optimiser->SetCostFunction(m_CostFunction);
  optimiser->SetInitialPosition(initialParameters);
  optimiser->SetNumberOfIterations(100);
  optimiser->SetGradientTolerance(0.0000001);
  optimiser->SetEpsilonFunction(0.0000001);
  optimiser->SetValueTolerance(0.0000001);

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearStereoExtrinsicsCalibration3DCostFunction::ParametersType finalParameters
      = optimiser->GetCurrentPosition();
  niftk::NonLinearStereoExtrinsicsCalibration3DCostFunction::MeasureType finalValues
      = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  counter = 0;
  if (m_CostFunction->GetOptimiseL2R())
  {
    leftToRightRotationVector.at<double>(0, 0) = finalParameters[counter++];
    leftToRightRotationVector.at<double>(0, 1) = finalParameters[counter++];
    leftToRightRotationVector.at<double>(0, 2) = finalParameters[counter++];
    cv::Rodrigues(leftToRightRotationVector, leftToRightRotationMatrix);

    leftToRightTranslationVector.at<double>(0, 0) = finalParameters[counter++];
    leftToRightTranslationVector.at<double>(1, 0) = finalParameters[counter++];
    leftToRightTranslationVector.at<double>(2, 0) = finalParameters[counter++];
  }
  if (m_CostFunction->GetOptimiseCameraExtrinsics())
  {
    for (int i = 0; i < rvecsLeft.size(); i++)
    {
      rvecsLeft[i].at<double>(0, 0) = finalParameters[counter++];
      rvecsLeft[i].at<double>(0, 1) = finalParameters[counter++];
      rvecsLeft[i].at<double>(0, 2) = finalParameters[counter++];
      tvecsLeft[i].at<double>(0, 0) = finalParameters[counter++];
      tvecsLeft[i].at<double>(0, 1) = finalParameters[counter++];
      tvecsLeft[i].at<double>(0, 2) = finalParameters[counter++];
    }
  }
  return finalRMS;
}

} // end namespace
