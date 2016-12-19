/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoHandEye3DCostFunction.h"
#include "niftkCalibrationUtilities_p.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoHandEye3DCostFunction::NonLinearStereoHandEye3DCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearStereoHandEye3DCostFunction::~NonLinearStereoHandEye3DCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearStereoHandEye3DCostFunction::MeasureType
NonLinearStereoHandEye3DCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  if (!this->GetUseHandMatrices() && m_HandMatrices->size() != m_Points->size())
  {
    niftkNiftyCalThrow() << "NonLinearStereoHandEye3DCostFunction requires hand (tracking) matrices.";
  }

  if (m_Points->size() != m_RightHandPoints->size())
  {
    niftkNiftyCalThrow() << "Different number of left and right point sets.";
  }

  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  // Notice how we are changin a parameter variable of length 6+6+6+6NDOF input into a 4+5+4+5+6+6N DOF array
  // in order to call the common underlying function ComputeStereoProjectionErrors.
  ParametersType internalParameters;
  internalParameters.SetSize(  4 // left intrinsic
                             + 5 // left distortion
                             + 4 // right intrinsic
                             + 5 // right distortion
                             + 6 // left to right
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
  internalParameters[9] = (*m_RightIntrinsic).at<double>(0, 0);
  internalParameters[10] = (*m_RightIntrinsic).at<double>(1, 1);
  internalParameters[11] = (*m_RightIntrinsic).at<double>(0, 2);
  internalParameters[12] = (*m_RightIntrinsic).at<double>(1, 2);
  internalParameters[13] = (*m_RightDistortion).at<double>(0, 0);
  internalParameters[14] = (*m_RightDistortion).at<double>(0, 1);
  internalParameters[15] = (*m_RightDistortion).at<double>(0, 2);
  internalParameters[16] = (*m_RightDistortion).at<double>(0, 3);
  internalParameters[17] = (*m_RightDistortion).at<double>(0, 4);
  internalParameters[18] = parameters[12];
  internalParameters[19] = parameters[13];
  internalParameters[20] = parameters[14];
  internalParameters[21] = parameters[15];
  internalParameters[22] = parameters[16];
  internalParameters[23] = parameters[17];

  cv::Mat handEyeRotationVector = cvCreateMat(1, 3, CV_64FC1);
  handEyeRotationVector.at<double>(0, 0) = parameters[0];
  handEyeRotationVector.at<double>(0, 1) = parameters[1];
  handEyeRotationVector.at<double>(0, 2) = parameters[2];

  cv::Mat handEyeTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  handEyeTranslationVector.at<double>(0, 0) = parameters[3];
  handEyeTranslationVector.at<double>(0, 1) = parameters[4];
  handEyeTranslationVector.at<double>(0, 2) = parameters[5];

  cv::Mat modelToWorldRotationVector = cvCreateMat(1, 3, CV_64FC1);
  modelToWorldRotationVector.at<double>(0, 0) = parameters[6];
  modelToWorldRotationVector.at<double>(0, 1) = parameters[7];
  modelToWorldRotationVector.at<double>(0, 2) = parameters[8];

  cv::Mat modelToWorldTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  modelToWorldTranslationVector.at<double>(0, 0) = parameters[9];
  modelToWorldTranslationVector.at<double>(0, 1) = parameters[10];
  modelToWorldTranslationVector.at<double>(0, 2) = parameters[11];

  cv::Matx44d modelToWorld = niftk::RodriguesToMatrix(modelToWorldRotationVector, modelToWorldTranslationVector);
  cv::Matx44d handEye = niftk::RodriguesToMatrix(handEyeRotationVector, handEyeTranslationVector);

  std::list<cv::Matx44d>::const_iterator matrixIter;
  unsigned int internalParameterCounter = 24;
  unsigned int parameterCounter = 18;

  for (matrixIter = m_HandMatrices->begin();
       matrixIter != m_HandMatrices->end();
       ++matrixIter
       )
  {
    cv::Mat trackingRotationVector = cvCreateMat(1, 3, CV_64FC1);
    trackingRotationVector.at<double>(0, 0) = parameters[parameterCounter++];
    trackingRotationVector.at<double>(0, 1) = parameters[parameterCounter++];
    trackingRotationVector.at<double>(0, 2) = parameters[parameterCounter++];

    cv::Mat trackingTranslationVector = cvCreateMat(1, 3, CV_64FC1);
    trackingTranslationVector.at<double>(0, 0) = parameters[parameterCounter++];
    trackingTranslationVector.at<double>(0, 1) = parameters[parameterCounter++];
    trackingTranslationVector.at<double>(0, 2) = parameters[parameterCounter++];

    cv::Matx44d handToWorld = niftk::RodriguesToMatrix(trackingRotationVector, trackingTranslationVector);
    cv::Matx44d worldToHand = handToWorld.inv();

    cv::Matx44d leftCameraMatrix = handEye * worldToHand * modelToWorld;

    cv::Mat rvec = cvCreateMat(1, 3, CV_64FC1);
    cv::Mat tvec = cvCreateMat(1, 3, CV_64FC1);
    niftk::MatrixToRodrigues(leftCameraMatrix, rvec, tvec);

    internalParameters[internalParameterCounter++] = rvec.at<double>(0, 0);
    internalParameters[internalParameterCounter++] = rvec.at<double>(0, 1);
    internalParameters[internalParameterCounter++] = rvec.at<double>(0, 2);
    internalParameters[internalParameterCounter++] = tvec.at<double>(0, 0);
    internalParameters[internalParameterCounter++] = tvec.at<double>(0, 1);
    internalParameters[internalParameterCounter++] = tvec.at<double>(0, 2);
  }

  niftk::ComputeStereoReconstructionErrors(m_Model, m_Points, m_RightHandPoints, internalParameters, result);
  return result;
}

} // end namespace
