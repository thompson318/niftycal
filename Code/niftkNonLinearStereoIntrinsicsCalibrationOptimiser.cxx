/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoIntrinsicsCalibrationOptimiser.h"
#include "niftkMatrixUtilities.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkNonLinearStereoIntrinsicsCalibrationCostFunction.h"
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibrationOptimiser::NonLinearStereoIntrinsicsCalibrationOptimiser()
{
  m_CostFunction = niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibrationOptimiser::~NonLinearStereoIntrinsicsCalibrationOptimiser()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibrationOptimiser::SetModelAndPoints(Model3D* const model,
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
void NonLinearStereoIntrinsicsCalibrationOptimiser::SetExtrinsics(std::vector<cv::Mat>* const rvecsLeft,
                                                                  std::vector<cv::Mat>* const tvecsLeft,
                                                                  cv::Mat* const leftToRightRotationMatrix,
                                                                  cv::Mat* const leftToRightTranslationVector
                                                                 )
{
  if (rvecsLeft == nullptr)
  {
    niftkNiftyCalThrow() << "Null left camera rotation vectors.";
  }

  if (tvecsLeft == nullptr)
  {
    niftkNiftyCalThrow() << "Null left camera translation vectors.";
  }

  if (leftToRightRotationMatrix == nullptr)
  {
    niftkNiftyCalThrow() << "Null leftToRightRotationMatrix.";
  }

  if (leftToRightTranslationVector == nullptr)
  {
    niftkNiftyCalThrow() << "Null leftToRightTranslationVector.";
  }

  if (leftToRightRotationMatrix->rows != 3 || leftToRightRotationMatrix->cols != 3)
  {
    niftkNiftyCalThrow() << "Left to Right rotation matrix should be 3x3, and its ("
                         << leftToRightRotationMatrix->cols << ", " << leftToRightRotationMatrix->rows << ")";
  }

  if (leftToRightTranslationVector->rows != 3 || leftToRightTranslationVector->cols != 1)
  {
    niftkNiftyCalThrow() << "Left to Right translation vector matrix should be 3x1, and its ("
                         << leftToRightTranslationVector->rows << ", " << leftToRightTranslationVector->cols << ")";
  }

  if (rvecsLeft->size() != tvecsLeft->size())
  {
    niftkNiftyCalThrow() << "Unequal extrinsic vectors: " << rvecsLeft->size()
                         << ", versus " << tvecsLeft->size();
  }

  m_CostFunction->SetExtrinsics(rvecsLeft, tvecsLeft, leftToRightRotationMatrix, leftToRightTranslationVector);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibrationOptimiser::SetDistortionParameters(cv::Mat* const leftDistortion,
                                                                            cv::Mat* const rightDistortion
                                                                           )
{
  if (leftDistortion == nullptr)
  {
    niftkNiftyCalThrow() << "Null left distortion parameters.";
  }

  if (rightDistortion == nullptr)
  {
    niftkNiftyCalThrow() << "Null right distortion parameters.";
  }

  if (leftDistortion->rows != 1 || leftDistortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Left distortion vector should be a 1x5 vector.";
  }

  if (rightDistortion->rows != 1 || rightDistortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Right distortion vector should be a 1x5 vector.";
  }

  m_CostFunction->SetDistortionParameters(leftDistortion, rightDistortion);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearStereoIntrinsicsCalibrationOptimiser::Optimise(cv::Mat& leftIntrinsic,
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

  niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::ParametersType initialParameters;
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
  optimiser->SetNumberOfIterations(1000);
  optimiser->SetGradientTolerance(0.0001);
  optimiser->SetEpsilonFunction(0.0001);
  optimiser->SetValueTolerance(0.0001);

  niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::MeasureType initialValues =
    m_CostFunction->GetValue(initialParameters);

  double initialRMS = m_CostFunction->GetRMS(initialValues);

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  for (int i = 0; i < finalParameters.GetSize(); i++)
  {
    std::cout << "NonLinearStereoIntrinsicsCalibrationOptimiser: final(" << i << ")=" << finalParameters[i]
                 << ", initial=" << initialParameters[i]
                 << ", diff=" << finalParameters[i] - initialParameters[i]
                 << std::endl;
  }

  counter = 0;
  leftIntrinsic.at<double>(0, 0) = finalParameters[counter++];
  leftIntrinsic.at<double>(1, 1) = finalParameters[counter++];
  leftIntrinsic.at<double>(0, 2) = finalParameters[counter++];
  leftIntrinsic.at<double>(1, 2) = finalParameters[counter++];
  rightIntrinsic.at<double>(0, 0) = finalParameters[counter++];
  rightIntrinsic.at<double>(1, 1) = finalParameters[counter++];
  rightIntrinsic.at<double>(0, 2) = finalParameters[counter++];
  rightIntrinsic.at<double>(1, 2) = finalParameters[counter++];

  std::cout << "NonLinearStereoIntrinsicsCalibrationOptimiser: initial rms=" << initialRMS
            << ", final rms=" << finalRMS
            << ", diff=" << finalRMS - initialRMS
            << std::endl;

  return finalRMS;
}

} // end namespace
