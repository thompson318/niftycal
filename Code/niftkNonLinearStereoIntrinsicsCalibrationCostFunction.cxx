/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoIntrinsicsCalibrationCostFunction.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkMatrixUtilities.h"
#include "niftkPointUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibrationCostFunction::NonLinearStereoIntrinsicsCalibrationCostFunction()
: m_RightHandPoints(nullptr)
, m_RvecsLeft(nullptr)
, m_TvecsLeft(nullptr)
, m_LeftToRightRotationMatrix(nullptr)
, m_LeftToRightTranslationVector(nullptr)
{
}


//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibrationCostFunction::~NonLinearStereoIntrinsicsCalibrationCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibrationCostFunction::SetRightHandPoints(std::list<PointSet>* const points)
{
  if (points == nullptr)
  {
    niftkNiftyCalThrow() << "Null right hand points.";
  }

  m_RightHandPoints = points;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibrationCostFunction::SetExtrinsics(std::vector<cv::Mat>* const rvecsLeft,
                                                                     std::vector<cv::Mat>* const tvecsLeft,
                                                                     cv::Mat* const leftToRightRotationMatrix,
                                                                     cv::Mat* const leftToRightTranslationVector
                                                                    )
{
  m_RvecsLeft = rvecsLeft;
  m_TvecsLeft = tvecsLeft;
  m_LeftToRightRotationMatrix = leftToRightRotationMatrix;
  m_LeftToRightTranslationVector = leftToRightTranslationVector;
  this->Modified();
}


//-----------------------------------------------------------------------------
unsigned int NonLinearStereoIntrinsicsCalibrationCostFunction::GetNumberOfValues(void) const
{
  return this->m_NumberOfValues;
}


//-----------------------------------------------------------------------------
NonLinearStereoIntrinsicsCalibrationCostFunction::MeasureType
NonLinearStereoIntrinsicsCalibrationCostFunction::InternalGetValue(const ParametersType& parameters) const
{
  if (m_Points->size() != m_RightHandPoints->size())
  {
    niftkNiftyCalThrow() << "Different number of left and right point sets.";
  }

  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  int counter = 0;
  cv::Mat leftIntrinsic = cv::Mat::zeros(3, 3, CV_64FC1);
  leftIntrinsic.at<double>(0, 0) = parameters[counter++];
  leftIntrinsic.at<double>(1, 1) = parameters[counter++];
  leftIntrinsic.at<double>(0, 2) = parameters[counter++];
  leftIntrinsic.at<double>(1, 2) = parameters[counter++];
  leftIntrinsic.at<double>(2, 2) = 1;

  cv::Mat leftDistortion = cv::Mat::zeros(1, 5, CV_64FC1);
  for (int i = 0; i < 5; i++)
  {
    leftDistortion.at<double>(0, i) = parameters[counter++];
  }

  cv::Mat rightIntrinsic = cv::Mat::zeros(3, 3, CV_64FC1);
  rightIntrinsic.at<double>(0, 0) = parameters[counter++];
  rightIntrinsic.at<double>(1, 1) = parameters[counter++];
  rightIntrinsic.at<double>(0, 2) = parameters[counter++];
  rightIntrinsic.at<double>(1, 2) = parameters[counter++];
  rightIntrinsic.at<double>(2, 2) = 1;

  cv::Mat rightDistortion = cv::Mat::zeros(1, 5, CV_64FC1);
  for (int i = 0; i < 5; i++)
  {
    rightDistortion.at<double>(0, i) = parameters[counter++];
  }

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
    niftk::Model3D triangulatedModelInLeftCameraSpace;

    niftk::TriangulatePointPairs(
      *leftViewIter,
      *rightViewIter,
      leftIntrinsic,
      leftDistortion,
      (*m_RvecsLeft)[numberOfViews],
      (*m_TvecsLeft)[numberOfViews],
      *m_LeftToRightRotationMatrix,
      *m_LeftToRightTranslationVector,
      rightIntrinsic,
      rightDistortion,
      triangulatedModelInLeftCameraSpace
    );

    cv::Matx44d modelToCamera = niftk::RodriguesToMatrix((*m_RvecsLeft)[numberOfViews], (*m_TvecsLeft)[numberOfViews]);
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
