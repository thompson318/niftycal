/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearMonoCameraCalibration3DCostFunction.h"
#include <Internal/niftkCalibrationUtilities_p.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearMonoCameraCalibration3DCostFunction::NonLinearMonoCameraCalibration3DCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearMonoCameraCalibration3DCostFunction::~NonLinearMonoCameraCalibration3DCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearMonoCameraCalibration3DCostFunction::SetIntrinsic(const cv::Mat& intrinsic)
{
  m_Intrinsic = intrinsic;
}


//-----------------------------------------------------------------------------
void NonLinearMonoCameraCalibration3DCostFunction::SetDistortion(const cv::Mat& distortion)
{
  m_Distortion = distortion;
}


//-----------------------------------------------------------------------------
NonLinearMonoCameraCalibration3DCostFunction::MeasureType
NonLinearMonoCameraCalibration3DCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  cv::Mat rightIntrinsic = cv::Mat::eye(3, 3, CV_64FC1);
  rightIntrinsic.at<double>(0, 0) = m_Intrinsic.at<double>(0, 0);
  rightIntrinsic.at<double>(1, 1) = m_Intrinsic.at<double>(1, 1);
  rightIntrinsic.at<double>(0, 2) = m_Intrinsic.at<double>(0, 2);
  rightIntrinsic.at<double>(1, 2) = m_Intrinsic.at<double>(1, 2);

  cv::Mat rightDistortion = cvCreateMat(1, 5, CV_64FC1);
  rightDistortion.at<double>(0, 0) = m_Distortion.at<double>(0, 0);
  rightDistortion.at<double>(0, 1) = m_Distortion.at<double>(0, 1);
  rightDistortion.at<double>(0, 2) = m_Distortion.at<double>(0, 2);
  rightDistortion.at<double>(0, 3) = m_Distortion.at<double>(0, 3);
  rightDistortion.at<double>(0, 4) = m_Distortion.at<double>(0, 4);

  itk::MultipleValuedCostFunction::MeasureType errorsPerView;
  unsigned long int errorCounter = 0;

  std::list<PointSet>::const_iterator leftPointsIter;
  std::list<PointSet>::const_iterator rightPointsIter;
  unsigned int leftCounter = 0;
  unsigned int rightCounter = 1;
  unsigned int parameterOffset = 0;

  for (leftPointsIter = m_Points->begin(),
       rightPointsIter = m_FakeRightHandPoints.begin();
       leftPointsIter != m_Points->end() &&
       rightPointsIter != m_FakeRightHandPoints.end();
       ++leftPointsIter,
       ++ rightPointsIter
      )
  {
    cv::Mat leftExtrinsicRVec = cvCreateMat(1, 3, CV_64FC1);
    leftExtrinsicRVec.at<double>(0, 0) = parameters[leftCounter*6 + 0 + parameterOffset];
    leftExtrinsicRVec.at<double>(0, 1) = parameters[leftCounter*6 + 1 + parameterOffset];
    leftExtrinsicRVec.at<double>(0, 2) = parameters[leftCounter*6 + 2 + parameterOffset];

    cv::Mat leftExtrinsicTVec = cvCreateMat(1, 3, CV_64FC1);
    leftExtrinsicTVec.at<double>(0, 0) = parameters[leftCounter*6 + 3 + parameterOffset];
    leftExtrinsicTVec.at<double>(0, 1) = parameters[leftCounter*6 + 4 + parameterOffset];
    leftExtrinsicTVec.at<double>(0, 2) = parameters[leftCounter*6 + 5 + parameterOffset];

    cv::Matx44d leftExtrinsic = niftk::RodriguesToMatrix(leftExtrinsicRVec, leftExtrinsicTVec);

    cv::Mat rightExtrinsicRVec = cvCreateMat(1, 3, CV_64FC1);
    rightExtrinsicRVec.at<double>(0, 0) = parameters[rightCounter*6 + 0 + parameterOffset];
    rightExtrinsicRVec.at<double>(0, 1) = parameters[rightCounter*6 + 1 + parameterOffset];
    rightExtrinsicRVec.at<double>(0, 2) = parameters[rightCounter*6 + 2 + parameterOffset];

    cv::Mat rightExtrinsicTVec = cvCreateMat(1, 3, CV_64FC1);
    rightExtrinsicTVec.at<double>(0, 0) = parameters[rightCounter*6 + 3 + parameterOffset];
    rightExtrinsicTVec.at<double>(0, 1) = parameters[rightCounter*6 + 4 + parameterOffset];
    rightExtrinsicTVec.at<double>(0, 2) = parameters[rightCounter*6 + 5 + parameterOffset];

    cv::Matx44d rightExtrinsic = niftk::RodriguesToMatrix(rightExtrinsicRVec, rightExtrinsicTVec);

    cv::Matx44d leftToRight = rightExtrinsic * leftExtrinsic.inv();

    cv::Mat leftToRightRotationVector = cvCreateMat(1, 3, CV_64FC1);
    cv::Mat leftToRightTranslationVector = cvCreateMat(3, 1, CV_64FC1);
    cv::Mat leftToRightRotationMatrix = cvCreateMat(3, 3, CV_64FC1);

    niftk::MatrixToRodrigues(leftToRight, leftToRightRotationVector, leftToRightTranslationVector);
    cv::Rodrigues(leftToRightRotationVector, leftToRightRotationMatrix);

    niftk::ComputeStereoReconstructionErrors(*m_Model,
                                             *leftPointsIter,
                                             *rightPointsIter,
                                             leftExtrinsic,
                                             m_Intrinsic,
                                             m_Distortion,
                                             leftToRightRotationMatrix,
                                             leftToRightTranslationVector,
                                             rightIntrinsic,
                                             rightDistortion,
                                             errorsPerView
                                             );

    for (unsigned long int i = 0; i < errorsPerView.size(); i++)
    {
      result[errorCounter++] = errorsPerView[i];
    }

    leftCounter++;
    rightCounter++;
    if (rightCounter == m_FakeRightHandPoints.size())
    {
      rightCounter = 0;
    }
  }
  return result;
}

} // end namespace
