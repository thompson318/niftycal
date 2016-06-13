/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearMaltiStereoHandEyeCostFunction.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkMatrixUtilities.h"
#include "niftkPointUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearMaltiStereoHandEyeCostFunction::NonLinearMaltiStereoHandEyeCostFunction()
: m_LeftIntrinsic(nullptr)
, m_LeftDistortion(nullptr)
, m_RightIntrinsic(nullptr)
, m_RightDistortion(nullptr)
, m_RightHandPoints(nullptr)
, m_NumberOfRightHandValues(0)
{
}


//-----------------------------------------------------------------------------
NonLinearMaltiStereoHandEyeCostFunction::~NonLinearMaltiStereoHandEyeCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearMaltiStereoHandEyeCostFunction::SetLeftIntrinsic(cv::Mat* const intrinsic)
{
  if (intrinsic->rows != 3 || intrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Intrinsic matrix should be 3x3, and its ("
                         << intrinsic->cols << ", " << intrinsic->rows << ")";
  }

  m_LeftIntrinsic = intrinsic;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMaltiStereoHandEyeCostFunction::SetLeftDistortion(cv::Mat* const distortion)
{
  if (distortion->rows != 1)
  {
    niftkNiftyCalThrow() << "Distortion vector should be a row vector.";
  }

  m_LeftDistortion = distortion;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMaltiStereoHandEyeCostFunction::SetRightIntrinsic(cv::Mat* const intrinsic)
{
  if (intrinsic->rows != 3 || intrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Intrinsic matrix should be 3x3, and its ("
                         << intrinsic->cols << ", " << intrinsic->rows << ")";
  }

  m_RightIntrinsic = intrinsic;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMaltiStereoHandEyeCostFunction::SetRightDistortion(cv::Mat* const distortion)
{
  if (distortion->rows != 1)
  {
    niftkNiftyCalThrow() << "Distortion vector should be a row vector.";
  }

  m_RightDistortion = distortion;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearMaltiStereoHandEyeCostFunction::SetRightHandPoints(std::list<PointSet>* const points)
{
  if (points == nullptr)
  {
    niftkNiftyCalThrow() << "Null right hand points.";
  }

  unsigned int num = 0;
  std::list<PointSet>::const_iterator iter;
  for (iter = points->begin();
       iter != points->end();
       ++iter
       )
  {
    num += (*iter).size();
  }

  m_NumberOfRightHandValues = num * 2; // For each point, we have deltaX and deltaY.
  m_RightHandPoints = points;
  this->Modified();
}


//-----------------------------------------------------------------------------
unsigned int NonLinearMaltiStereoHandEyeCostFunction::GetNumberOfValues(void) const
{
  return m_NumberOfValues + m_NumberOfRightHandValues;
}


//-----------------------------------------------------------------------------
void NonLinearMaltiStereoHandEyeCostFunction::ProjectPoints(const PointSet& points,
                                                            const cv::Matx44d& extrinsic,
                                                            const cv::Mat& intrinsic,
                                                            const cv::Mat& distortion,
                                                            MeasureType& values,
                                                            unsigned int& totalPointCounter
                                                           ) const
{
  std::vector<cv::Point2f> observed(points.size());
  std::vector<cv::Point2f> projected(points.size());
  std::vector<niftk::NiftyCalIdType> ids(points.size());

  niftk::ProjectMatchingPoints(*m_Model,
                               points,
                               extrinsic,
                               intrinsic,
                               distortion,
                               observed,
                               projected,
                               ids
                              );

  for (unsigned int i = 0; i < observed.size(); i++)
  {
    values[totalPointCounter++] = (observed[i].x - projected[i].x);
    values[totalPointCounter++] = (observed[i].y - projected[i].y);
  }
}


//-----------------------------------------------------------------------------
NonLinearMaltiStereoHandEyeCostFunction::MeasureType
NonLinearMaltiStereoHandEyeCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  if (m_Points->size() != m_RightHandPoints->size())
  {
    niftkNiftyCalThrow() << "Different number of left and right point sets.";
  }

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

  cv::Mat stereoExtrinsicsRotationVector = cvCreateMat(1, 3, CV_64FC1);
  stereoExtrinsicsRotationVector.at<double>(0, 0) = parameters[12];
  stereoExtrinsicsRotationVector.at<double>(0, 1) = parameters[13];
  stereoExtrinsicsRotationVector.at<double>(0, 2) = parameters[14];

  cv::Mat stereoExtrinsicsTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  stereoExtrinsicsTranslationVector.at<double>(0, 0) = parameters[15];
  stereoExtrinsicsTranslationVector.at<double>(0, 1) = parameters[16];
  stereoExtrinsicsTranslationVector.at<double>(0, 2) = parameters[17];

  cv::Matx44d modelToWorld = niftk::RodriguesToMatrix(modelToWorldRotationVector, modelToWorldTranslationVector);
  cv::Matx44d handEye = niftk::RodriguesToMatrix(handEyeRotationVector, handEyeTranslationVector);
  cv::Matx44d stereoExtrinsics = niftk::RodriguesToMatrix(stereoExtrinsicsRotationVector,
                                                          stereoExtrinsicsTranslationVector);

  std::list<PointSet>::const_iterator leftViewIter;
  std::list<PointSet>::const_iterator rightViewIter;

  unsigned int totalPointCounter = 0;
  unsigned int matrixParametersCounter = 18;

  for (leftViewIter = m_Points->begin(),
       rightViewIter = m_RightHandPoints->begin();
       leftViewIter != m_Points->end()
       && rightViewIter != m_RightHandPoints->end();
       ++leftViewIter,
       ++rightViewIter
       )
  {

    cv::Mat trackingRotationVector = cvCreateMat(1, 3, CV_64FC1);
    trackingRotationVector.at<double>(0, 0) = parameters[matrixParametersCounter++];
    trackingRotationVector.at<double>(0, 1) = parameters[matrixParametersCounter++];
    trackingRotationVector.at<double>(0, 2) = parameters[matrixParametersCounter++];

    cv::Mat trackingTranslationVector = cvCreateMat(1, 3, CV_64FC1);
    trackingTranslationVector.at<double>(0, 0) = parameters[matrixParametersCounter++];
    trackingTranslationVector.at<double>(0, 1) = parameters[matrixParametersCounter++];
    trackingTranslationVector.at<double>(0, 2) = parameters[matrixParametersCounter++];

    cv::Matx44d handToWorld = niftk::RodriguesToMatrix(trackingRotationVector, trackingTranslationVector);
    cv::Matx44d worldToHand = handToWorld.inv();

    cv::Matx44d leftCameraMatrix = handEye * worldToHand * modelToWorld;
    cv::Matx44d rightCameraMatrix = stereoExtrinsics * leftCameraMatrix;

    this->ProjectPoints(*leftViewIter, leftCameraMatrix,
                        *m_LeftIntrinsic, *m_LeftDistortion,
                        result, totalPointCounter);

    this->ProjectPoints(*rightViewIter, rightCameraMatrix,
                        *m_RightIntrinsic, *m_RightDistortion,
                        result, totalPointCounter);
  }

  return result;
}

} // end namespace
