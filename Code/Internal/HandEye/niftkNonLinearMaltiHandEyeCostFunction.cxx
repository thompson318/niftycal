/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearMaltiHandEyeCostFunction.h"
#include <Internal/niftkCalibrationUtilities_p.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearMaltiHandEyeCostFunction::NonLinearMaltiHandEyeCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearMaltiHandEyeCostFunction::~NonLinearMaltiHandEyeCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearMaltiHandEyeCostFunction::MeasureType
NonLinearMaltiHandEyeCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  if (!this->GetUseHandMatrices() && m_HandMatrices->size() != m_Points->size())
  {
    niftkNiftyCalThrow() << "NonLinearMaltiHandEyeCostFunction requires hand (tracking) matrices.";
  }

  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  // Notice how we are extending a fixed length, 21DOF input into a 4+5+6N DOF array
  // in order to call the common underlying function ComputeMonoProjectionErrors.
  ParametersType internalParameters;
  internalParameters.SetSize(  4 // intrinsic
                             + 5 // distortion
                             + 6*m_Points->size() // extrinsic = 6DOF x N views.
                            );
  internalParameters.Fill(0);

  // Copy first 9 parameters = 4 intrinsic, 5 distortion
  for (int i = 0; i < 9; i++)
  {
    internalParameters[i] = parameters[i];
  }

  cv::Mat handEyeRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  handEyeRotationVector.at<double>(0, 0) = parameters[9];
  handEyeRotationVector.at<double>(0, 1) = parameters[10];
  handEyeRotationVector.at<double>(0, 2) = parameters[11];

  cv::Mat handEyeTranslationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  handEyeTranslationVector.at<double>(0, 0) = parameters[12];
  handEyeTranslationVector.at<double>(0, 1) = parameters[13];
  handEyeTranslationVector.at<double>(0, 2) = parameters[14];

  cv::Mat modelToWorldRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  modelToWorldRotationVector.at<double>(0, 0) = parameters[15];
  modelToWorldRotationVector.at<double>(0, 1) = parameters[16];
  modelToWorldRotationVector.at<double>(0, 2) = parameters[17];

  cv::Mat modelToWorldTranslationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  modelToWorldTranslationVector.at<double>(0, 0) = parameters[18];
  modelToWorldTranslationVector.at<double>(0, 1) = parameters[19];
  modelToWorldTranslationVector.at<double>(0, 2) = parameters[20];

  cv::Matx44d modelToWorld = niftk::RodriguesToMatrix(modelToWorldRotationVector, modelToWorldTranslationVector);
  cv::Matx44d handEye = niftk::RodriguesToMatrix(handEyeRotationVector, handEyeTranslationVector);

  std::list<cv::Matx44d>::const_iterator matrixIter;
  unsigned int parameterCounter = 9;

  for (matrixIter = m_HandMatrices->begin();
       matrixIter != m_HandMatrices->end();
       ++matrixIter
       )
  {
    cv::Matx44d handToWorld = (*matrixIter);
    cv::Matx44d worldToHand = handToWorld.inv();
    cv::Matx44d cameraMatrix = handEye * worldToHand * modelToWorld;

    cv::Mat rvec = cv::Mat::zeros(1, 3, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64FC1);
    niftk::MatrixToRodrigues(cameraMatrix, rvec, tvec);

    internalParameters[parameterCounter++] = rvec.at<double>(0, 0);
    internalParameters[parameterCounter++] = rvec.at<double>(0, 1);
    internalParameters[parameterCounter++] = rvec.at<double>(0, 2);
    internalParameters[parameterCounter++] = tvec.at<double>(0, 0);
    internalParameters[parameterCounter++] = tvec.at<double>(0, 1);
    internalParameters[parameterCounter++] = tvec.at<double>(0, 2);
  }

  niftk::ComputeMonoProjectionErrors(m_Model, m_Points, internalParameters, result);
  return result;
}

} // end namespace
