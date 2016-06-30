/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoExtrinsicsCalibrationOptimiser.h"
#include "niftkMatrixUtilities.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkNonLinearStereoExtrinsicsCalibrationCostFunction.h"
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibrationOptimiser::NonLinearStereoExtrinsicsCalibrationOptimiser()
{
  m_CostFunction = niftk::NonLinearStereoExtrinsicsCalibrationCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibrationOptimiser::~NonLinearStereoExtrinsicsCalibrationOptimiser()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibrationOptimiser::SetModelAndPoints(Model3D* const model,
                                                            std::list<PointSet>* const leftPoints,
                                                            std::list<PointSet>* const rightPoints
                                                           )
{
  m_CostFunction->SetModel(model);
  m_CostFunction->SetPoints(leftPoints);
  m_CostFunction->SetRightHandPoints(rightPoints);

  unsigned long int numberOfTriangulatablePoints = 0;
  std::list<PointSet>::const_iterator leftViewIter;
  std::list<PointSet>::const_iterator rightViewIter;
  niftk::PointSet::const_iterator pointIter;

  for (leftViewIter = leftPoints->begin(),
       rightViewIter = rightPoints->begin();
       leftViewIter != leftPoints->end() && rightViewIter != rightPoints->end();
       ++leftViewIter,
       ++rightViewIter
       )
  {
    for (pointIter = leftViewIter->begin();
         pointIter != leftViewIter->end();
         ++pointIter
         )
    {
      niftk::NiftyCalIdType id = (*pointIter).first;
      if (rightViewIter->find(id) != rightViewIter->end()
          && model->find(id) != model->end()
          )
      {
        numberOfTriangulatablePoints++;
      }
    }
  }
  m_CostFunction->SetNumberOfValues(numberOfTriangulatablePoints * 3);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibrationOptimiser::SetIntrinsics(cv::Mat* const leftIntrinsic,
                                                                  cv::Mat* const leftDistortion,
                                                                  cv::Mat* const rightIntrinsic,
                                                                  cv::Mat* const rightDistortion
                                                                 )
{
  if (leftIntrinsic->rows != 3 || leftIntrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Left intrinsic matrix should be 3x3, and its ("
                         << leftIntrinsic->cols << ", " << leftIntrinsic->rows << ")";
  }

  if (leftDistortion->rows != 1 || leftDistortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Left distortion vector should be a 1x5 vector.";
  }

  if (rightIntrinsic->rows != 3 || rightIntrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Right intrinsic matrix should be 3x3, and its ("
                         << rightIntrinsic->cols << ", " << rightIntrinsic->rows << ")";
  }

  if (rightDistortion->rows != 1 || rightDistortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Right distortion vector should be a 1x5 vector.";
  }

  m_CostFunction->SetIntrinsics(leftIntrinsic, leftDistortion,
                                rightIntrinsic, rightDistortion);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearStereoExtrinsicsCalibrationOptimiser::Optimise(std::vector<cv::Mat>& rvecsLeft,
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

  niftk::NonLinearStereoExtrinsicsCalibrationCostFunction::ParametersType initialParameters;
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
  optimiser->SetNumberOfIterations(1000);
  optimiser->SetGradientTolerance(0.0001);
  optimiser->SetEpsilonFunction(0.0001);
  optimiser->SetValueTolerance(0.0001);

  niftk::NonLinearStereoExtrinsicsCalibrationCostFunction::MeasureType initialValues =
    m_CostFunction->GetValue(initialParameters);

  double initialRMS = m_CostFunction->GetRMS(initialValues);
  int startingPointOfExtrinsics = 6;
  std::cout << "NonLinearStereoExtrinsicsCalibrationOptimiser: initial rms=" << initialRMS << std::endl;

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearStereoExtrinsicsCalibrationCostFunction::ParametersType finalParameters
      = optimiser->GetCurrentPosition();
  niftk::NonLinearStereoExtrinsicsCalibrationCostFunction::MeasureType finalValues
      = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  for (int i = startingPointOfExtrinsics; i < finalParameters.GetSize(); i++)
  {
    std::cout << "NonLinearStereoExtrinsicsCalibrationOptimiser: final(" << i << ")=" << finalParameters[i]
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
    std::cout << "NonLinearStereoExtrinsicsCalibrationOptimiser: final(" << i << ")=" << finalParameters[i]
                 << ", initial=" << initialParameters[i]
                 << ", diff=" << finalParameters[i] - initialParameters[i]
                 << std::endl;
  }

  std::cout << "NonLinearStereoExtrinsicsCalibrationOptimiser: stereo="
            << leftToRightRotationVector.at<double>(0, 0) << ", "
            << leftToRightRotationVector.at<double>(0, 1) << ", "
            << leftToRightRotationVector.at<double>(0, 2) << ", "
            << leftToRightTranslationVector.at<double>(0, 0) << ", "
            << leftToRightTranslationVector.at<double>(0, 1) << ", "
            << leftToRightTranslationVector.at<double>(0, 2) << std::endl;
  std::cout << "NonLinearStereoExtrinsicsCalibrationOptimiser: rms=" << finalRMS << std::endl;

  return finalRMS;
}

} // end namespace
