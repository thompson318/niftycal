/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoCameraCalibration2DOptimiser.h"
#include "niftkCalibrationUtilities_p.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration2DOptimiser::NonLinearStereoCameraCalibration2DOptimiser()
: m_CostFunction(nullptr)
, m_Optimise2DOFStereo(false)
, m_ForceUnitVectorAxes(false)
{
  m_CostFunction = niftk::NonLinearStereoCameraCalibration2DCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration2DOptimiser::~NonLinearStereoCameraCalibration2DOptimiser()
{

}


//-----------------------------------------------------------------------------
void NonLinearStereoCameraCalibration2DOptimiser::SetOptimise2DOFStereo(const bool& optimise)
{
  m_Optimise2DOFStereo = optimise;
  m_CostFunction->SetOptimise2DOFStereo(optimise);
  this->Modified();
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

  int numberOfParameters = 6 * rvecsLeft.size(); // extrinsics of each left hand camera
  if (m_Optimise2DOFStereo)
  {
    numberOfParameters += 2;
  }
  else
  {
    numberOfParameters += 6;
  }

  niftk::NonLinearStereoCameraCalibration2DCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(numberOfParameters);

  int counter = 0;
  cv::Vec3d translationUnitVector = 0;
  cv::Vec3d rotationUnitVector = 0;
  cv::Matx14d axisAngle = 0;

  if (m_Optimise2DOFStereo)
  {
    axisAngle = niftk::RodriguesToAxisAngle(leftToRightRotationVector);
    initialParameters[counter++] = axisAngle(0, 3); // angle in radians.
    initialParameters[counter++] = cv::norm(leftToRightTranslationVector); // magnitude of trans.

    translationUnitVector[0] = leftToRightTranslationVector.at<double>(0, 0) / initialParameters[1];
    translationUnitVector[1] = leftToRightTranslationVector.at<double>(0, 1) / initialParameters[1];
    translationUnitVector[2] = leftToRightTranslationVector.at<double>(0, 2) / initialParameters[1];

    if (m_ForceUnitVectorAxes)
    {
      int maxIndex = niftk::GetMajorAxisIndex(translationUnitVector);
      int sign = niftk::Signum(translationUnitVector[maxIndex]);
      translationUnitVector = 0;
      translationUnitVector[maxIndex] = sign;      
    }
    m_CostFunction->SetTranslationVector(translationUnitVector);

    rotationUnitVector[0] = axisAngle(0, 0);
    rotationUnitVector[1] = axisAngle(0, 1);
    rotationUnitVector[2] = axisAngle(0, 2);

    if (m_ForceUnitVectorAxes)
    {
      int maxIndex = niftk::GetMajorAxisIndex(rotationUnitVector);
      int sign = niftk::Signum(rotationUnitVector[maxIndex]);
      rotationUnitVector = 0;
      rotationUnitVector[maxIndex] = sign;
    }
    m_CostFunction->SetAxisOfRotation(rotationUnitVector);
  }
  else
  {
    initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 0);
    initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 1);
    initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 2);
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(0, 0);
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(0, 1);
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(0, 2);
  }
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
  if (m_Optimise2DOFStereo)
  {
    axisAngle(0, 0) = rotationUnitVector[0];
    axisAngle(0, 1) = rotationUnitVector[1];
    axisAngle(0, 2) = rotationUnitVector[2];
    axisAngle(0, 3) = finalParameters[counter++];

    leftToRightRotationVector = niftk::AxisAngleToRodrigues(axisAngle);

    double translationInMillimetres = finalParameters[counter++];

    leftToRightTranslationVector.at<double>(0, 0) = translationUnitVector[0] * translationInMillimetres;
    leftToRightTranslationVector.at<double>(0, 1) = translationUnitVector[1] * translationInMillimetres;
    leftToRightTranslationVector.at<double>(0, 2) = translationUnitVector[2] * translationInMillimetres;
  }
  else
  {
    leftToRightRotationVector.at<double>(0, 0) = finalParameters[counter++];
    leftToRightRotationVector.at<double>(0, 1) = finalParameters[counter++];
    leftToRightRotationVector.at<double>(0, 2) = finalParameters[counter++];
    leftToRightTranslationVector.at<double>(0, 0) = finalParameters[counter++];
    leftToRightTranslationVector.at<double>(0, 1) = finalParameters[counter++];
    leftToRightTranslationVector.at<double>(0, 2) = finalParameters[counter++];
  }
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
                                                             cv::Mat& rightIntrinsic,
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

  if (rightIntrinsic.rows != 3 || rightIntrinsic.cols != 3)
  {
    niftkNiftyCalThrow() << "Right intrinsic matrix should be 3x3, and its ("
                         << rightIntrinsic.cols << ", " << rightIntrinsic.rows << ")";
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
                            + 4                    // right intrinsic
                            + 3                    // leftToRightRotationVector
                            + 3                    // leftToRightTranslationVector
                            + 6 * rvecsLeft.size() // extrinsics of each left hand camera
                           );

  int counter = 0;
  initialParameters[counter++] = leftIntrinsic.at<double>(0, 0);
  initialParameters[counter++] = leftIntrinsic.at<double>(1, 1);
  initialParameters[counter++] = leftIntrinsic.at<double>(0, 2);
  initialParameters[counter++] = leftIntrinsic.at<double>(1, 2);
  initialParameters[counter++] = rightIntrinsic.at<double>(0, 0);
  initialParameters[counter++] = rightIntrinsic.at<double>(1, 1);
  initialParameters[counter++] = rightIntrinsic.at<double>(0, 2);
  initialParameters[counter++] = rightIntrinsic.at<double>(1, 2);
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

  // Do optimisation.
  optimiser->StartOptimization();

  // Get final parameters.
  niftk::NonLinearStereoCameraCalibration2DCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  niftk::NonLinearStereoCameraCalibration2DCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
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

} // end namespace
