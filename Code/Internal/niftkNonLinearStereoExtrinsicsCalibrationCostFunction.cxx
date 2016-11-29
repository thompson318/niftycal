/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoExtrinsicsCalibrationCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibrationCostFunction::NonLinearStereoExtrinsicsCalibrationCostFunction()
: m_LeftIntrinsic(nullptr)
, m_LeftDistortion(nullptr)
, m_RightIntrinsic(nullptr)
, m_RightDistortion(nullptr)
{
}


//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibrationCostFunction::~NonLinearStereoExtrinsicsCalibrationCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibrationCostFunction::SetIntrinsics(cv::Mat* const leftIntrinsic,
                                                                     cv::Mat* const rightIntrinsic
                                                                    )
{
  if (leftIntrinsic->rows != 3 || leftIntrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Left intrinsic matrix should be 3x3, and its ("
                         << leftIntrinsic->cols << ", " << leftIntrinsic->rows << ")";
  }

  if (rightIntrinsic->rows != 3 || rightIntrinsic->cols != 3)
  {
    niftkNiftyCalThrow() << "Right intrinsic matrix should be 3x3, and its ("
                         << rightIntrinsic->cols << ", " << rightIntrinsic->rows << ")";
  }

  m_LeftIntrinsic = leftIntrinsic;
  m_RightIntrinsic = rightIntrinsic;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibrationCostFunction::SetDistortionParameters(cv::Mat* const leftDistortion,
                                                                               cv::Mat* const rightDistortion
                                                                              )
{
  if (leftDistortion->rows != 1 || leftDistortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Left distortion vector should be a 1x5 vector.";
  }

  if (rightDistortion->rows != 1 || rightDistortion->cols != 5)
  {
    niftkNiftyCalThrow() << "Right distortion vector should be a 1x5 vector.";
  }

  m_LeftDistortion = leftDistortion;
  m_RightDistortion = rightDistortion;
  this->Modified();
}


//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibrationCostFunction::MeasureType
NonLinearStereoExtrinsicsCalibrationCostFunction::InternalGetValue(const ParametersType& parameters) const
{
  if (m_Points->size() != m_RightHandPoints->size())
  {
    niftkNiftyCalThrow() << "Different number of left and right point sets.";
  }

  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  int counter = 0;
  cv::Mat leftToRightRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  leftToRightRotationVector.at<double>(0, 0) = parameters[counter++];
  leftToRightRotationVector.at<double>(0, 1) = parameters[counter++];
  leftToRightRotationVector.at<double>(0, 2) = parameters[counter++];

  cv::Mat leftToRightRotationMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Rodrigues(leftToRightRotationVector, leftToRightRotationMatrix);

  cv::Mat leftToRightTranslationVector = cvCreateMat(3, 1, CV_64FC1);
  leftToRightTranslationVector.at<double>(0, 0) = parameters[counter++];
  leftToRightTranslationVector.at<double>(1, 0) = parameters[counter++];
  leftToRightTranslationVector.at<double>(2, 0) = parameters[counter++];

  int numberOfViews = 0;
  unsigned long int pointCounter = 0;

  std::list<PointSet>::const_iterator leftViewIter;
  std::list<PointSet>::const_iterator rightViewIter;

  for (leftViewIter = m_Points->begin(),
       rightViewIter = m_RightHandPoints->begin();
       leftViewIter != m_Points->end() && rightViewIter != m_RightHandPoints->end();
       ++leftViewIter,
       ++rightViewIter
       )
  {
    cv::Mat leftCameraRotationVector = cvCreateMat(1, 3, CV_64FC1);
    leftCameraRotationVector.at<double>(0, 0) = parameters[counter++];
    leftCameraRotationVector.at<double>(0, 1) = parameters[counter++];
    leftCameraRotationVector.at<double>(0, 2) = parameters[counter++];

    cv::Mat leftCameraTranslationVector = cvCreateMat(1, 3, CV_64FC1);
    leftCameraTranslationVector.at<double>(0, 0) = parameters[counter++];
    leftCameraTranslationVector.at<double>(0, 1) = parameters[counter++];
    leftCameraTranslationVector.at<double>(0, 2) = parameters[counter++];

    niftk::Model3D triangulatedModelInLeftCameraSpace;

    niftk::TriangulatePointPairs(
      *leftViewIter,
      *rightViewIter,
      *m_LeftIntrinsic,
      *m_LeftDistortion,
      leftCameraRotationVector,
      leftCameraTranslationVector,
      leftToRightRotationMatrix,
      leftToRightTranslationVector,
      *m_RightIntrinsic,
      *m_RightDistortion,
      triangulatedModelInLeftCameraSpace
    );

    cv::Matx44d modelToCamera = niftk::RodriguesToMatrix(leftCameraRotationVector, leftCameraTranslationVector);
    cv::Matx44d cameraToModel = modelToCamera.inv(cv::DECOMP_SVD);

    niftk::Model3D triangulatedModelInModelSpace = niftk::TransformModel(triangulatedModelInLeftCameraSpace,
                                                                         cameraToModel);

    niftk::Model3D::const_iterator modelIter;
    for (modelIter = triangulatedModelInModelSpace.begin();
         modelIter != triangulatedModelInModelSpace.end();
         ++modelIter
         )
    {
      niftk::NiftyCalIdType id = (*modelIter).first;
      Model3D::const_iterator goldIter = m_Model->find(id);
      if (goldIter == m_Model->end())
      {
        niftkNiftyCalThrow() << "Failed to find point " << id << " in gold standard model.";
      }
      niftk::Point3D triangulatedPoint = (*modelIter).second;
      niftk::Point3D goldStandardPoint = (*goldIter).second;
      result[pointCounter++] = triangulatedPoint.point.x - goldStandardPoint.point.x;
      result[pointCounter++] = triangulatedPoint.point.y - goldStandardPoint.point.y;
      result[pointCounter++] = triangulatedPoint.point.z - goldStandardPoint.point.z;
    }
    numberOfViews++;
  }
  return result;
}

} // end namespace
