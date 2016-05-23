/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearMaltiNDOFHandEyeOptimiser.h"
#include "niftkMatrixUtilities.h"
#include "niftkNiftyCalExceptionMacro.h"
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearMaltiNDOFHandEyeOptimiser::NonLinearMaltiNDOFHandEyeOptimiser()
{
  m_CostFunction = niftk::NonLinearMaltiNDOFHandEyeCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearMaltiNDOFHandEyeOptimiser::~NonLinearMaltiNDOFHandEyeOptimiser()
{

}


//-----------------------------------------------------------------------------
void NonLinearMaltiNDOFHandEyeOptimiser::SetModel(Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMaltiNDOFHandEyeOptimiser::SetPoints(std::list<PointSet>* const points)
{
  m_CostFunction->SetPoints(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMaltiNDOFHandEyeOptimiser::SetHandMatrices(std::list<cv::Matx44d>* const matrices)
{
  m_CostFunction->SetHandMatrices(matrices);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMaltiNDOFHandEyeOptimiser::SetIntrinsic(cv::Mat* const intrinsic)
{
  m_CostFunction->SetIntrinsic(intrinsic);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMaltiNDOFHandEyeOptimiser::SetDistortion(cv::Mat* const distortion)
{
  m_CostFunction->SetDistortion(distortion);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearMaltiNDOFHandEyeOptimiser::Optimise(cv::Matx44d& modelToWorld,
                                                    cv::Matx44d& handEye
                                                    )
{
  std::list<cv::Matx44d>* matrices = m_CostFunction->GetHandMatrices();
  if (matrices == nullptr)
  {
    niftkNiftyCalThrow() << "Hand (tracking) matrices are null.";
  }

  cv::Mat modelToWorldRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat modelToWorldTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(modelToWorld, modelToWorldRotationVector, modelToWorldTranslationVector);

  cv::Mat handEyeRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat handEyeTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(handEye, handEyeRotationVector, handEyeTranslationVector);

  niftk::NonLinearMaltiNDOFHandEyeCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(  6                    // model to world
                            + 6                    // hand eye
                            + 6 * matrices->size() // extrinsics for each view.
                           );

  // Set initial parameters.
  initialParameters[0] = handEyeRotationVector.at<double>(0, 0);
  initialParameters[1] = handEyeRotationVector.at<double>(0, 1);
  initialParameters[2] = handEyeRotationVector.at<double>(0, 2);
  initialParameters[3] = handEyeTranslationVector.at<double>(0, 0);
  initialParameters[4] = handEyeTranslationVector.at<double>(0, 1);
  initialParameters[5] = handEyeTranslationVector.at<double>(0, 2);
  initialParameters[6] = modelToWorldRotationVector.at<double>(0, 0);
  initialParameters[7] = modelToWorldRotationVector.at<double>(0, 1);
  initialParameters[8] = modelToWorldRotationVector.at<double>(0, 2);
  initialParameters[9] = modelToWorldTranslationVector.at<double>(0, 0);
  initialParameters[10] = modelToWorldTranslationVector.at<double>(0, 1);
  initialParameters[11] = modelToWorldTranslationVector.at<double>(0, 2);

  unsigned int counter = 12;
  std::list<cv::Matx44d>::const_iterator iter;
  for (iter = (*matrices).begin();
       iter != (*matrices).end();
       ++iter
       )
  {
    cv::Mat trackingRotationVector = cvCreateMat(1, 3, CV_64FC1);
    cv::Mat trackingTranslationVector = cvCreateMat(1, 3, CV_64FC1);
    niftk::MatrixToRodrigues(*iter, trackingRotationVector, trackingTranslationVector);

    initialParameters[counter++] = trackingRotationVector.at<double>(0, 0);
    initialParameters[counter++] = trackingRotationVector.at<double>(0, 1);
    initialParameters[counter++] = trackingRotationVector.at<double>(0, 2);
    initialParameters[counter++] = trackingTranslationVector.at<double>(0, 0);
    initialParameters[counter++] = trackingTranslationVector.at<double>(0, 1);
    initialParameters[counter++] = trackingTranslationVector.at<double>(0, 2);
  }

  m_CostFunction->SetNumberOfParameters(initialParameters.GetSize());

  // Setup optimiser.
  itk::LevenbergMarquardtOptimizer::Pointer optimiser = itk::LevenbergMarquardtOptimizer::New();
  optimiser->UseCostFunctionGradientOff(); // use default VNL derivative, not our one.
  optimiser->SetCostFunction(m_CostFunction);
  optimiser->SetInitialPosition(initialParameters);
  optimiser->SetNumberOfIterations(20000000);
  optimiser->SetGradientTolerance(0.000000005);
  optimiser->SetEpsilonFunction(0.000000005);
  optimiser->SetValueTolerance(0.000000005);

  niftk::NonLinearMaltiNDOFHandEyeCostFunction::MeasureType initialValues = m_CostFunction->GetValue(initialParameters);
  double initialRMS = m_CostFunction->GetRMS(initialValues);

  for (int i = 12; i < initialParameters.GetSize(); i++)
  {
    std::cout << "NonLinearMaltiNDOFHandEyeOptimiser: initial(" << i << ")=" << initialParameters[i] << std::endl;
    if ((i - 12) % 6 == 5)
    {
      std::cout << std::endl;
    }
  }
  std::cout << ", rms=" << initialRMS << std::endl;

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearMaltiNDOFHandEyeCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  handEyeRotationVector.at<double>(0, 0) = finalParameters[0];
  handEyeRotationVector.at<double>(0, 1) = finalParameters[1];
  handEyeRotationVector.at<double>(0, 2) = finalParameters[2];
  handEyeTranslationVector.at<double>(0, 0) = finalParameters[3];
  handEyeTranslationVector.at<double>(0, 1) = finalParameters[4];
  handEyeTranslationVector.at<double>(0, 2) = finalParameters[5];
  modelToWorldRotationVector.at<double>(0, 0) = finalParameters[6];
  modelToWorldRotationVector.at<double>(0, 1) = finalParameters[7];
  modelToWorldRotationVector.at<double>(0, 2) = finalParameters[8];
  modelToWorldTranslationVector.at<double>(0, 0) = finalParameters[9];
  modelToWorldTranslationVector.at<double>(0, 1) = finalParameters[10];
  modelToWorldTranslationVector.at<double>(0, 2) = finalParameters[11];

  modelToWorld = niftk::RodriguesToMatrix(modelToWorldRotationVector, modelToWorldTranslationVector);
  handEye = niftk::RodriguesToMatrix(handEyeRotationVector, handEyeTranslationVector);

  niftk::NonLinearMaltiNDOFHandEyeCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  for (int i = 12; i < finalParameters.GetSize(); i++)
  {
    std::cout << "NonLinearMaltiNDOFHandEyeOptimiser: final(" << i << ")=" << finalParameters[i]
                 << ", initial=" << initialParameters[i]
                 << ", diff=" << finalParameters[i] - initialParameters[i]
                 << std::endl;
    if ((i - 12) % 6 == 5)
    {
      std::cout << std::endl;
    }
  }
  std::cout << "NonLinearMaltiNDOFHandEyeOptimiser: hand-eye="
            << handEyeRotationVector.at<double>(0, 0) << ", "
            << handEyeRotationVector.at<double>(0, 1) << ", "
            << handEyeRotationVector.at<double>(0, 2) << ", "
            << handEyeTranslationVector.at<double>(0, 0) << ", "
            << handEyeTranslationVector.at<double>(0, 1) << ", "
            << handEyeTranslationVector.at<double>(0, 2) << std::endl;
  std::cout << "NonLinearMaltiNDOFHandEyeOptimiser: model-to-world="
            << modelToWorldRotationVector.at<double>(0, 0) << ", "
            << modelToWorldRotationVector.at<double>(0, 1) << ", "
            << modelToWorldRotationVector.at<double>(0, 2) << ", "
            << modelToWorldTranslationVector.at<double>(0, 0) << ", "
            << modelToWorldTranslationVector.at<double>(0, 1) << ", "
            << modelToWorldTranslationVector.at<double>(0, 2) << std::endl;

  std::cout << "NonLinearMaltiNDOFHandEyeOptimiser: rms=" << finalRMS << std::endl;

  return finalRMS;
}

} // end namespace
