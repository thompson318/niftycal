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
  m_CostFunction->SetDistortionParameters(leftDistortion,
                                          rightDistortion
                                         );
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibration3DOptimiser::SetIntrinsics(cv::Mat* const leftIntrinsic,
                                                                  cv::Mat* const rightIntrinsic
                                                                 )
{
  m_CostFunction->SetIntrinsics(leftIntrinsic,
                                rightIntrinsic
                               );
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

  niftk::NonLinearStereoExtrinsicsCalibration3DCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(  3                    // leftToRightRotationVector
                            + 3                    // leftToRightTranslationVector
                            + 6 * rvecsLeft.size() // extrinsics of each left hand camera
                           );

  // Set initial parameters.
  int counter = 0;
  initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 0);
  initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 1);
  initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 2);
  initialParameters[counter++] = leftToRightTranslationVector.at<double>(0, 0);
  initialParameters[counter++] = leftToRightTranslationVector.at<double>(0, 1);
  initialParameters[counter++] = leftToRightTranslationVector.at<double>(0, 2);
  for (int i = 0; i < rvecsLeft.size(); i++)
  {
    initialParameters[counter++] = rvecsLeft[i].at<double>(0, 0);
    initialParameters[counter++] = rvecsLeft[i].at<double>(0, 1);
    initialParameters[counter++] = rvecsLeft[i].at<double>(0, 2);
    initialParameters[counter++] = tvecsLeft[i].at<double>(0, 0);
    initialParameters[counter++] = tvecsLeft[i].at<double>(0, 1);
    initialParameters[counter++] = tvecsLeft[i].at<double>(0, 2);
  }

  m_CostFunction->SetNumberOfParameters(initialParameters.GetSize());

  // Setup optimiser.
  itk::LevenbergMarquardtOptimizer::Pointer optimiser = itk::LevenbergMarquardtOptimizer::New();
  optimiser->UseCostFunctionGradientOff(); // use default VNL derivative, not our one.
  optimiser->SetCostFunction(m_CostFunction);
  optimiser->SetInitialPosition(initialParameters);
  optimiser->SetNumberOfIterations(100);
  optimiser->SetGradientTolerance(0.001);
  optimiser->SetEpsilonFunction(0.001);
  optimiser->SetValueTolerance(0.001);

  niftk::NonLinearStereoExtrinsicsCalibration3DCostFunction::MeasureType initialValues =
    m_CostFunction->GetValue(initialParameters);

  double initialRMS = m_CostFunction->GetRMS(initialValues);
  int startingPointOfExtrinsics = 6;
  std::cout << "NonLinearStereoExtrinsicsCalibration3DOptimiser: initial rms=" << initialRMS << std::endl;

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearStereoExtrinsicsCalibration3DCostFunction::ParametersType finalParameters
      = optimiser->GetCurrentPosition();
  niftk::NonLinearStereoExtrinsicsCalibration3DCostFunction::MeasureType finalValues
      = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  for (int i = startingPointOfExtrinsics; i < finalParameters.GetSize(); i++)
  {
    std::cout << "NonLinearStereoExtrinsicsCalibration3DOptimiser: final(" << i << ")=" << finalParameters[i]
                 << ", initial=" << initialParameters[i]
                 << ", diff=" << finalParameters[i] - initialParameters[i]
                 << std::endl;
    if ((i - startingPointOfExtrinsics) % 6 == 5)
    {
      std::cout << std::endl;
    }
  }

  counter = 0;
  leftToRightRotationVector.at<double>(0, 0) = finalParameters[counter++];
  leftToRightRotationVector.at<double>(0, 1) = finalParameters[counter++];
  leftToRightRotationVector.at<double>(0, 2) = finalParameters[counter++];
  leftToRightTranslationVector.at<double>(0, 0) = finalParameters[counter++];
  leftToRightTranslationVector.at<double>(0, 1) = finalParameters[counter++];
  leftToRightTranslationVector.at<double>(0, 2) = finalParameters[counter++];
  for (int i = 0; i < rvecsLeft.size(); i++)
  {
    rvecsLeft[i].at<double>(0, 0) = finalParameters[counter++];
    rvecsLeft[i].at<double>(0, 1) = finalParameters[counter++];
    rvecsLeft[i].at<double>(0, 2) = finalParameters[counter++];
    tvecsLeft[i].at<double>(0, 0) = finalParameters[counter++];
    tvecsLeft[i].at<double>(0, 1) = finalParameters[counter++];
    tvecsLeft[i].at<double>(0, 2) = finalParameters[counter++];
  }

  cv::Rodrigues(leftToRightRotationVector, leftToRightRotationMatrix);

  for (int i = 0; i < startingPointOfExtrinsics; i++)
  {
    std::cout << "NonLinearStereoExtrinsicsCalibration3DOptimiser: final(" << i << ")=" << finalParameters[i]
                 << ", initial=" << initialParameters[i]
                 << ", diff=" << finalParameters[i] - initialParameters[i]
                 << std::endl;
  }

  std::cout << "NonLinearStereoExtrinsicsCalibration3DOptimiser: stereo="
            << leftToRightRotationVector.at<double>(0, 0) << ", "
            << leftToRightRotationVector.at<double>(0, 1) << ", "
            << leftToRightRotationVector.at<double>(0, 2) << ", "
            << leftToRightTranslationVector.at<double>(0, 0) << ", "
            << leftToRightTranslationVector.at<double>(0, 1) << ", "
            << leftToRightTranslationVector.at<double>(0, 2) << std::endl;
  std::cout << "NonLinearStereoExtrinsicsCalibration3DOptimiser: rms=" << finalRMS << std::endl;

  return finalRMS;
}

} // end namespace
