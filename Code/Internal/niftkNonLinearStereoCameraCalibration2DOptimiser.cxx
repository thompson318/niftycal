/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoCameraCalibration2DOptimiser.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration2DOptimiser::NonLinearStereoCameraCalibration2DOptimiser()
{
  m_CostFunction = niftk::NonLinearStereoCameraCalibration2DCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration2DOptimiser::~NonLinearStereoCameraCalibration2DOptimiser()
{

}


//-----------------------------------------------------------------------------
void NonLinearStereoCameraCalibration2DOptimiser::SetModelAndPoints(const Model3D* const model,
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
void NonLinearStereoCameraCalibration2DOptimiser::SetIntrinsic(const cv::Mat* const intrinsic)
{
  m_CostFunction->SetIntrinsic(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoCameraCalibration2DOptimiser::SetDistortion(const cv::Mat* const distortion)
{
  m_CostFunction->SetDistortion(distortion);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoCameraCalibration2DOptimiser::SetRightIntrinsic(const cv::Mat* const intrinsic)
{
  m_CostFunction->SetRightIntrinsic(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoCameraCalibration2DOptimiser::SetRightDistortion(const cv::Mat* const distortion)
{
  m_CostFunction->SetRightDistortion(distortion);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearStereoCameraCalibration2DOptimiser::Optimise(std::vector<cv::Mat>& rvecsLeft,
                                                             std::vector<cv::Mat>& tvecsLeft,
                                                             cv::Mat& leftToRightRotationMatrix,
                                                             cv::Mat& leftToRightTranslationVector
                                                            )
{
  cv::Mat leftToRightRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);

  niftk::NonLinearStereoCameraCalibration2DCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(  3                    // leftToRightRotationVector
                            + 3                    // leftToRightTranslationVector
                            + 6 * rvecsLeft.size() // extrinsics of each left hand camera
                           );

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

  m_CostFunction->SetOptimiseIntrinsics(false);
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

  optimiser->StartOptimization();

  niftk::NonLinearStereoCameraCalibration2DCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  niftk::NonLinearStereoCameraCalibration2DCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

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

  return finalRMS;
}


//-----------------------------------------------------------------------------
double NonLinearStereoCameraCalibration2DOptimiser::Optimise(cv::Mat& leftIntrinsic,
                                                             cv::Mat& leftDistortion,
                                                             cv::Mat& rightIntrinsic,
                                                             cv::Mat& rightDistortion,
                                                             std::vector<cv::Mat>& rvecsLeft,
                                                             std::vector<cv::Mat>& tvecsLeft,
                                                             cv::Mat& leftToRightRotationMatrix,
                                                             cv::Mat& leftToRightTranslationVector
                                                            )
{
  if (leftIntrinsic.rows != 3 || leftIntrinsic.cols != 3)
  {
    niftkNiftyCalThrow() << "Left intrinsic matrix should be 3x3, and its ("
                         << leftIntrinsic.cols << ", " << leftIntrinsic.rows << ")";
  }

  if (leftDistortion.rows != 1 || leftDistortion.cols != 5)
  {
    niftkNiftyCalThrow() << "Left distortion vector should be a 1x5 vector.";
  }

  if (rightIntrinsic.rows != 3 || rightIntrinsic.cols != 3)
  {
    niftkNiftyCalThrow() << "Right intrinsic matrix should be 3x3, and its ("
                         << rightIntrinsic.cols << ", " << rightIntrinsic.rows << ")";
  }

  if (rightDistortion.rows != 1 || rightDistortion.cols != 5)
  {
    niftkNiftyCalThrow() << "Right distortion vector should be a 1x5 vector.";
  }

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

  niftk::NonLinearStereoCameraCalibration2DCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(  4                    // left intrinsic
                            + 5                    // left distortion
                            + 4                    // right intrinsic
                            + 5                    // right distortion
                            + 3                    // leftToRightRotationVector
                            + 3                    // leftToRightTranslationVector
                            + 6 * rvecsLeft.size() // extrinsics of each left hand camera
                           );

  int counter = 0;
  initialParameters[counter++] = leftIntrinsic.at<double>(0, 0);
  initialParameters[counter++] = leftIntrinsic.at<double>(1, 1);
  initialParameters[counter++] = leftIntrinsic.at<double>(0, 2);
  initialParameters[counter++] = leftIntrinsic.at<double>(1, 2);
  for (int i = 0; i < leftDistortion.cols; i++)
  {
    initialParameters[counter++] = leftDistortion.at<double>(0, i);
  }
  initialParameters[counter++] = rightIntrinsic.at<double>(0, 0);
  initialParameters[counter++] = rightIntrinsic.at<double>(1, 1);
  initialParameters[counter++] = rightIntrinsic.at<double>(0, 2);
  initialParameters[counter++] = rightIntrinsic.at<double>(1, 2);
  for (int i = 0; i < rightDistortion.cols; i++)
  {
    initialParameters[counter++] = rightDistortion.at<double>(0, i);
  }
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

  m_CostFunction->SetOptimiseIntrinsics(true);
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
  niftk::NonLinearStereoCameraCalibration2DCostFunction::MeasureType initialValues =
    m_CostFunction->GetValue(initialParameters);


  double initialRMS = m_CostFunction->GetRMS(initialValues);
  int startingPointOfExtrinsics = 4 + leftDistortion.cols + 4 + rightDistortion.cols + 6;

  for (int i = 0; i < startingPointOfExtrinsics; i++)
  {
    std::cout << "NonLinearStereoCameraCalibration2DOptimiser: initial(" << i << ")=" << initialParameters[i] << std::endl;;
  }
  for (int i = startingPointOfExtrinsics; i < initialParameters.GetSize(); i++)
  {
    std::cout << "NonLinearStereoCameraCalibration2DOptimiser: initial(" << i << ")=" << initialParameters[i] << std::endl;
    if ((i - startingPointOfExtrinsics) % 6 == 5)
    {
      std::cout << std::endl;
    }
  }

  std::cout << "NonLinearStereoCameraCalibration2DOptimiser: initial rms=" << initialRMS << std::endl;
*/

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearStereoCameraCalibration2DCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  niftk::NonLinearStereoCameraCalibration2DCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);
/*
  for (int i = startingPointOfExtrinsics; i < finalParameters.GetSize(); i++)
  {
    std::cout << "NonLinearStereoCameraCalibration2DOptimiser: final(" << i << ")=" << finalParameters[i]
                 << ", initial=" << initialParameters[i]
                 << ", diff=" << finalParameters[i] - initialParameters[i]
                 << std::endl;
    if ((i - startingPointOfExtrinsics) % 6 == 5)
    {
      std::cout << std::endl;
    }
  }
*/
  counter = 0;
  leftIntrinsic.at<double>(0, 0) = finalParameters[counter++];
  leftIntrinsic.at<double>(1, 1) = finalParameters[counter++];
  leftIntrinsic.at<double>(0, 2) = finalParameters[counter++];
  leftIntrinsic.at<double>(1, 2) = finalParameters[counter++];
  for (int i = 0; i < leftDistortion.cols; i++)
  {
    leftDistortion.at<double>(0, i) = finalParameters[counter++];
  }
  rightIntrinsic.at<double>(0, 0) = finalParameters[counter++];
  rightIntrinsic.at<double>(1, 1) = finalParameters[counter++];
  rightIntrinsic.at<double>(0, 2) = finalParameters[counter++];
  rightIntrinsic.at<double>(1, 2) = finalParameters[counter++];
  for (int i = 0; i < rightDistortion.cols; i++)
  {
    rightDistortion.at<double>(0, i) = finalParameters[counter++];
  }
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
/*
  for (int i = 0; i < startingPointOfExtrinsics; i++)
  {
    std::cout << "NonLinearStereoCameraCalibration2DOptimiser: final(" << i << ")=" << finalParameters[i]
                 << ", initial=" << initialParameters[i]
                 << ", diff=" << finalParameters[i] - initialParameters[i]
                 << std::endl;
  }

  std::cout << "NonLinearStereoCameraCalibration2DOptimiser: stereo="
            << leftToRightRotationVector.at<double>(0, 0) << ", "
            << leftToRightRotationVector.at<double>(0, 1) << ", "
            << leftToRightRotationVector.at<double>(0, 2) << ", "
            << leftToRightTranslationVector.at<double>(0, 0) << ", "
            << leftToRightTranslationVector.at<double>(0, 1) << ", "
            << leftToRightTranslationVector.at<double>(0, 2) << std::endl;
  std::cout << "NonLinearStereoCameraCalibration2DOptimiser: left="
            << leftIntrinsic.at<double>(0, 0) << ", "
            << leftIntrinsic.at<double>(1, 1) << ", "
            << leftIntrinsic.at<double>(0, 2) << ", "
            << leftIntrinsic.at<double>(1, 2) << ", ";
  for (int i = 0; i < leftDistortion.cols; i++)
  {
    std::cout << leftDistortion.at<double>(0, i) << ", ";
    if (i == leftDistortion.cols - 1)
    {
      std::cout << std::endl;
    }
  }
  std::cout << "NonLinearStereoCameraCalibration2DOptimiser: right="
            << rightIntrinsic.at<double>(0, 0) << ", "
            << rightIntrinsic.at<double>(1, 1) << ", "
            << rightIntrinsic.at<double>(0, 2) << ", "
            << rightIntrinsic.at<double>(1, 2) << ", ";
  for (int i = 0; i < leftDistortion.cols; i++)
  {
    std::cout << rightDistortion.at<double>(0, i) << ", ";
    if (i == rightDistortion.cols - 1)
    {
      std::cout << std::endl;
    }
  }
  std::cout << "NonLinearStereoCameraCalibration2DOptimiser: rms=" << finalRMS << std::endl;
*/
  return finalRMS;
}

} // end namespace
