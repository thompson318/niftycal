/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoExtrinsicsCalibrationCostFunction.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkMatrixUtilities.h"
#include "niftkPointUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoExtrinsicsCalibrationCostFunction::NonLinearStereoExtrinsicsCalibrationCostFunction()
: m_RightHandPoints(nullptr)
, m_LeftIntrinsic(nullptr)
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
void NonLinearStereoExtrinsicsCalibrationCostFunction::SetRightHandPoints(std::list<PointSet>* const points)
{
  if (points == nullptr)
  {
    niftkNiftyCalThrow() << "Null right hand points.";
  }

  m_RightHandPoints = points;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoExtrinsicsCalibrationCostFunction::SetIntrinsics(cv::Mat* const leftIntrinsic,
                                                                     cv::Mat* const leftDistortion,
                                                                     cv::Mat* const rightIntrinsic,
                                                                     cv::Mat* const rightDistortion
                                                                    )
{
  m_LeftIntrinsic = leftIntrinsic;
  m_LeftDistortion = leftDistortion;
  m_RightIntrinsic = rightIntrinsic;
  m_RightDistortion = rightDistortion;
  this->Modified();
}


//-----------------------------------------------------------------------------
unsigned int NonLinearStereoExtrinsicsCalibrationCostFunction::GetNumberOfValues(void) const
{
  return this->m_NumberOfValues;
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

  cv::Mat leftToRightTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  leftToRightTranslationVector.at<double>(0, 0) = parameters[counter++];
  leftToRightTranslationVector.at<double>(0, 1) = parameters[counter++];
  leftToRightTranslationVector.at<double>(0, 2) = parameters[counter++];

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
      Model3D::iterator goldIter = m_Model->find(id);
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
