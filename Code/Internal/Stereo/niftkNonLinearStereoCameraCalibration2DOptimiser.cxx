/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoCameraCalibration2DOptimiser.h"
#include <Internal/niftkCalibrationUtilities_p.h>
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoCameraCalibration2DOptimiser::NonLinearStereoCameraCalibration2DOptimiser()
: m_CostFunction(nullptr)
, m_OptimiseIntrinsics(false)
, m_OptimiseExtrinsics(true)
, m_OptimiseR2L(true)
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
void NonLinearStereoCameraCalibration2DOptimiser::SetOptimiseIntrinsics(const bool& optimise)
{
  m_OptimiseIntrinsics = optimise;
  m_CostFunction->SetOptimiseIntrinsics(optimise);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoCameraCalibration2DOptimiser::SetOptimiseExtrinsics(const bool& optimise)
{
  m_OptimiseExtrinsics = optimise;
  m_CostFunction->SetOptimiseExtrinsics(optimise);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoCameraCalibration2DOptimiser::SetOptimiseR2L(const bool& optimise)
{
  m_OptimiseR2L = optimise;
  m_CostFunction->SetOptimiseR2L(optimise);
  this->Modified();
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
void NonLinearStereoCameraCalibration2DOptimiser::SetExtrinsics(
  const std::vector<cv::Mat>& rvecsLeft,
  const std::vector<cv::Mat>& tvecsLeft)
{
  m_CostFunction->SetExtrinsics(rvecsLeft, tvecsLeft);
  this->Modified();
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

  if (m_Optimise2DOFStereo && m_OptimiseR2L)
  {
    niftkNiftyCalThrow() << "Optimise2DOFStereo and OptimiseR2L are mutually exclusive";
  }

  if (m_ForceUnitVectorAxes && m_OptimiseR2L)
  {
    niftkNiftyCalThrow() << "ForceUnitVectorAxes and OptimiseR2L are mutually exclusive";
  }

  cv::Mat leftToRightRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  niftk::SafeRodrigues(leftToRightRotationMatrix, leftToRightRotationVector);

  int numberOfParameters = 0;

  if (m_OptimiseIntrinsics)
  {
    numberOfParameters += 8;
  }
  if (m_Optimise2DOFStereo)
  {
    numberOfParameters += 2;
  }
  if (m_OptimiseR2L)
  {
    numberOfParameters += 6;
  }
  if (m_OptimiseExtrinsics)
  {
    numberOfParameters += 6 * rvecsLeft.size(); // extrinsics of each left hand camera
  }

  if (numberOfParameters == 0)
  {
    niftkNiftyCalThrow() << "Nothing to optimise";
  }

  niftk::NonLinearStereoCameraCalibration2DCostFunction::ParametersType initialParameters;
  initialParameters.SetSize(numberOfParameters);

  int counter = 0;
  cv::Vec3d translationUnitVector = 0;
  cv::Vec3d rotationUnitVector = 0;
  double angle = 0;
  cv::Matx14d axisAngle = 0;

  if (m_OptimiseIntrinsics)
  {
    initialParameters[counter++] = leftIntrinsic.at<double>(0, 0);
    initialParameters[counter++] = leftIntrinsic.at<double>(1, 1);
    initialParameters[counter++] = leftIntrinsic.at<double>(0, 2);
    initialParameters[counter++] = leftIntrinsic.at<double>(1, 2);
    initialParameters[counter++] = rightIntrinsic.at<double>(0, 0);
    initialParameters[counter++] = rightIntrinsic.at<double>(1, 1);
    initialParameters[counter++] = rightIntrinsic.at<double>(0, 2);
    initialParameters[counter++] = rightIntrinsic.at<double>(1, 2);
  }

  if (m_Optimise2DOFStereo)
  {
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(0, 0); // tx
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(1, 0); // ty

    translationUnitVector[0] = leftToRightTranslationVector.at<double>(0, 0);
    translationUnitVector[1] = leftToRightTranslationVector.at<double>(1, 0);
    translationUnitVector[2] = leftToRightTranslationVector.at<double>(2, 0);

    axisAngle = niftk::RodriguesToAxisAngle(leftToRightRotationVector);
    rotationUnitVector[0] = axisAngle(0, 0);
    rotationUnitVector[1] = axisAngle(0, 1);
    rotationUnitVector[2] = axisAngle(0, 2);
    angle = axisAngle(0, 3);

    if (m_ForceUnitVectorAxes)
    {
      translationUnitVector[2] = 0;
      rotationUnitVector[0] = 0;
      rotationUnitVector[1] = -1;
      rotationUnitVector[2] = 0;
      angle = 0;
    }

    m_CostFunction->SetTranslationVector(translationUnitVector);
    m_CostFunction->SetAxisOfRotation(rotationUnitVector);
    m_CostFunction->SetAngleOfRotation(angle);
  }
  if (m_OptimiseR2L)
  {
    initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 0);
    initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 1);
    initialParameters[counter++] = leftToRightRotationVector.at<double>(0, 2);
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(0, 0);
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(1, 0);
    initialParameters[counter++] = leftToRightTranslationVector.at<double>(2, 0);
  }
  else
  {
    axisAngle = niftk::RodriguesToAxisAngle(leftToRightRotationVector);
    rotationUnitVector[0] = axisAngle(0, 0);
    rotationUnitVector[1] = axisAngle(0, 1);
    rotationUnitVector[2] = axisAngle(0, 2);
    m_CostFunction->SetTranslationVector(leftToRightTranslationVector);
    m_CostFunction->SetAngleOfRotation(axisAngle(0, 3));
    m_CostFunction->SetAxisOfRotation(rotationUnitVector);
  }
  if (m_OptimiseExtrinsics)
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
  else
  {
    m_CostFunction->SetExtrinsics(rvecsLeft, tvecsLeft);
  }

  m_CostFunction->SetNumberOfParameters(initialParameters.GetSize());

  // Setup optimiser.
  itk::LevenbergMarquardtOptimizer::Pointer optimiser = itk::LevenbergMarquardtOptimizer::New();
  optimiser->UseCostFunctionGradientOff(); // use default VNL derivative, not our one.
  optimiser->SetCostFunction(m_CostFunction);
  optimiser->SetInitialPosition(initialParameters);
  optimiser->SetNumberOfIterations(100);
  optimiser->SetGradientTolerance(0.0000001);
  optimiser->SetEpsilonFunction(0.0000001);
  optimiser->SetValueTolerance(0.0000001);

  optimiser->StartOptimization();

  niftk::NonLinearStereoCameraCalibration2DCostFunction::ParametersType finalParameters = optimiser->GetCurrentPosition();
  niftk::NonLinearStereoCameraCalibration2DCostFunction::MeasureType finalValues = m_CostFunction->GetValue(finalParameters);
  double finalRMS = m_CostFunction->GetRMS(finalValues);

  counter = 0;
  if (m_OptimiseIntrinsics)
  {
    leftIntrinsic.at<double>(0, 0) = finalParameters[counter++];
    leftIntrinsic.at<double>(1, 1) = finalParameters[counter++];
    leftIntrinsic.at<double>(0, 2) = finalParameters[counter++];
    leftIntrinsic.at<double>(1, 2) = finalParameters[counter++];
    rightIntrinsic.at<double>(0, 0) = finalParameters[counter++];
    rightIntrinsic.at<double>(1, 1) = finalParameters[counter++];
    rightIntrinsic.at<double>(0, 2) = finalParameters[counter++];
    rightIntrinsic.at<double>(1, 2) = finalParameters[counter++];
  }
  if (m_Optimise2DOFStereo)
  {
    if (m_ForceUnitVectorAxes)
    {
      leftToRightTranslationVector.at<double>(2, 0) = 0;
    }

    axisAngle(0, 0) = rotationUnitVector[0];
    axisAngle(0, 1) = rotationUnitVector[1];
    axisAngle(0, 2) = rotationUnitVector[2];
    axisAngle(0, 3) = angle;

    leftToRightRotationVector = niftk::AxisAngleToRodrigues(axisAngle);
    leftToRightTranslationVector.at<double>(0, 0) = finalParameters[counter++];
    leftToRightTranslationVector.at<double>(1, 0) = finalParameters[counter++];
  }
  if (m_OptimiseR2L)
  {
    leftToRightRotationVector.at<double>(0, 0) = finalParameters[counter++];
    leftToRightRotationVector.at<double>(0, 1) = finalParameters[counter++];
    leftToRightRotationVector.at<double>(0, 2) = finalParameters[counter++];
    leftToRightTranslationVector.at<double>(0, 0) = finalParameters[counter++];
    leftToRightTranslationVector.at<double>(1, 0) = finalParameters[counter++];
    leftToRightTranslationVector.at<double>(2, 0) = finalParameters[counter++];
  }
  if (m_OptimiseExtrinsics)
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

  cv::Rodrigues(leftToRightRotationVector, leftToRightRotationMatrix);

  return finalRMS;
}

} // end namespace
