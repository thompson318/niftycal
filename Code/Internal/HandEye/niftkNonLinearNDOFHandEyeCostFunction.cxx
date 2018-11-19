/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearNDOFHandEyeCostFunction.h"
#include <Internal/niftkCalibrationUtilities_p.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearNDOFHandEyeCostFunction::NonLinearNDOFHandEyeCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearNDOFHandEyeCostFunction::~NonLinearNDOFHandEyeCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearNDOFHandEyeCostFunction::MeasureType
NonLinearNDOFHandEyeCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  if (!this->GetUseHandMatrices() && m_HandMatrices->size() != m_Points->size())
  {
    niftkNiftyCalThrow() << "NonLinearNDOFHandEyeCostFunction requires hand (tracking) matrices.";
  }

  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  // Notice how we are changin a parameter variable of length 12+6NDOF input into a 4+5+6N DOF array
  // in order to call the common underlying function ComputeMonoProjectionErrors.
  ParametersType internalParameters;
  internalParameters.SetSize(  4 // intrinsic
                             + 5 // distortion
                             + (m_Points->size()*6)
                            );
  internalParameters.Fill(0);

  // Intrinsic params are not in the input array, as they are constant.
  internalParameters[0] = (*m_Intrinsic).at<double>(0, 0);
  internalParameters[1] = (*m_Intrinsic).at<double>(1, 1);
  internalParameters[2] = (*m_Intrinsic).at<double>(0, 2);
  internalParameters[3] = (*m_Intrinsic).at<double>(1, 2);
  internalParameters[4] = (*m_Distortion).at<double>(0, 0);
  internalParameters[5] = (*m_Distortion).at<double>(0, 1);
  internalParameters[6] = (*m_Distortion).at<double>(0, 2);
  internalParameters[7] = (*m_Distortion).at<double>(0, 3);
  internalParameters[8] = (*m_Distortion).at<double>(0, 4);

  cv::Mat handEyeRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  handEyeRotationVector.at<double>(0, 0) = parameters[0];
  handEyeRotationVector.at<double>(0, 1) = parameters[1];
  handEyeRotationVector.at<double>(0, 2) = parameters[2];

  cv::Mat handEyeTranslationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  handEyeTranslationVector.at<double>(0, 0) = parameters[3];
  handEyeTranslationVector.at<double>(0, 1) = parameters[4];
  handEyeTranslationVector.at<double>(0, 2) = parameters[5];

  cv::Mat modelToWorldRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  modelToWorldRotationVector.at<double>(0, 0) = parameters[6];
  modelToWorldRotationVector.at<double>(0, 1) = parameters[7];
  modelToWorldRotationVector.at<double>(0, 2) = parameters[8];

  cv::Mat modelToWorldTranslationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  modelToWorldTranslationVector.at<double>(0, 0) = parameters[9];
  modelToWorldTranslationVector.at<double>(0, 1) = parameters[10];
  modelToWorldTranslationVector.at<double>(0, 2) = parameters[11];

  cv::Matx44d modelToWorld = niftk::RodriguesToMatrix(modelToWorldRotationVector, modelToWorldTranslationVector);
  cv::Matx44d handEye = niftk::RodriguesToMatrix(handEyeRotationVector, handEyeTranslationVector);

  std::list<cv::Matx44d>::const_iterator matrixIter;
  unsigned int internalParameterCounter = 9;
  unsigned int parameterCounter = 12;

  for (matrixIter = m_HandMatrices->begin();
       matrixIter != m_HandMatrices->end();
       ++matrixIter
       )
  {
    cv::Mat rvec = cv::Mat::zeros(1, 3, CV_64FC1);
    rvec.at<double>(0, 0) = parameters[parameterCounter++];
    rvec.at<double>(0, 1) = parameters[parameterCounter++];
    rvec.at<double>(0, 2) = parameters[parameterCounter++];

    cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64FC1);
    tvec.at<double>(0, 0) = parameters[parameterCounter++];
    tvec.at<double>(0, 1) = parameters[parameterCounter++];
    tvec.at<double>(0, 2) = parameters[parameterCounter++];

    cv::Matx44d handToWorld = niftk::RodriguesToMatrix(rvec, tvec);
    cv::Matx44d worldToHand = handToWorld.inv();
    cv::Matx44d cameraMatrix = handEye * worldToHand * modelToWorld;

    niftk::MatrixToRodrigues(cameraMatrix, rvec, tvec);

    internalParameters[internalParameterCounter++] = rvec.at<double>(0, 0);
    internalParameters[internalParameterCounter++] = rvec.at<double>(0, 1);
    internalParameters[internalParameterCounter++] = rvec.at<double>(0, 2);
    internalParameters[internalParameterCounter++] = tvec.at<double>(0, 0);
    internalParameters[internalParameterCounter++] = tvec.at<double>(0, 1);
    internalParameters[internalParameterCounter++] = tvec.at<double>(0, 2);
  }

  niftk::ComputeMonoProjectionErrors(m_Model, m_Points, internalParameters, result);
  return result;
}

} // end namespace
