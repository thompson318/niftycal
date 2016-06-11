/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearMalti12DOFHandEyeCostFunction.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkMatrixUtilities.h"
#include "niftkPointUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearMalti12DOFHandEyeCostFunction::NonLinearMalti12DOFHandEyeCostFunction()
: m_Intrinsic(nullptr)
, m_Distortion(nullptr)
{
}


//-----------------------------------------------------------------------------
NonLinearMalti12DOFHandEyeCostFunction::~NonLinearMalti12DOFHandEyeCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearMalti12DOFHandEyeCostFunction::SetIntrinsic(cv::Mat* const intrinsic)
{
  if (intrinsic->rows != 3 || intrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Intrinsic matrix should be 3x3, and its ("
                         << intrinsic->cols << ", " << intrinsic->rows << ")";
  }

  m_Intrinsic = intrinsic;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMalti12DOFHandEyeCostFunction::SetDistortion(cv::Mat* const distortion)
{
  if (distortion->rows != 1)
  {
    niftkNiftyCalThrow() << "Distortion vector should be a row vector.";
  }

  m_Distortion = distortion;
  this->Modified();
}


//-----------------------------------------------------------------------------
NonLinearMalti12DOFHandEyeCostFunction::MeasureType
NonLinearMalti12DOFHandEyeCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

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

  unsigned int totalPointCounter = 0;

  std::list<PointSet>::const_iterator viewIter;
  std::list<cv::Matx44d>::const_iterator matrixIter;

  for (viewIter = m_Points->begin(),
       matrixIter = m_HandMatrices->begin();
       viewIter != m_Points->end()
       && matrixIter != m_HandMatrices->end();
       ++viewIter,
       ++matrixIter
       )
  {
    cv::Matx44d handToWorld = (*matrixIter);
    cv::Matx44d worldToHand = handToWorld.inv();
    cv::Matx44d cameraMatrix = handEye * worldToHand * modelToWorld;

    std::vector<cv::Point2f> observed((*viewIter).size());
    std::vector<cv::Point2f> projected((*viewIter).size());

    niftk::ProjectMatchingPoints(*m_Model,
                                 *viewIter,
                                 cameraMatrix,
                                 *m_Intrinsic,
                                 *m_Distortion,
                                 observed,
                                 projected
                                );

    for (unsigned int i = 0; i < observed.size(); i++)
    {
      result[totalPointCounter++] = (observed[i].x - projected[i].x);
      result[totalPointCounter++] = (observed[i].y - projected[i].y);
    }
  }

  return result;
}

} // end namespace
