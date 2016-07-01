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
: m_RvecsLeft(nullptr)
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
void NonLinearStereoIntrinsicsCalibrationCostFunction::SetExtrinsics(std::vector<cv::Mat>* const rvecsLeft,
                                                                     std::vector<cv::Mat>* const tvecsLeft,
                                                                     cv::Mat* const leftToRightRotationMatrix,
                                                                     cv::Mat* const leftToRightTranslationVector
                                                                    )
{
  if (rvecsLeft == nullptr)
  {
    niftkNiftyCalThrow() << "Null left camera rotation vectors.";
  }

  if (tvecsLeft == nullptr)
  {
    niftkNiftyCalThrow() << "Null left camera translation vectors.";
  }

  if (leftToRightRotationMatrix == nullptr)
  {
    niftkNiftyCalThrow() << "Null leftToRightRotationMatrix.";
  }

  if (leftToRightTranslationVector == nullptr)
  {
    niftkNiftyCalThrow() << "Null leftToRightTranslationVector.";
  }

  if (leftToRightRotationMatrix->rows != 3 || leftToRightRotationMatrix->cols != 3)
  {
    niftkNiftyCalThrow() << "Left to Right rotation matrix should be 3x3, and its ("
                         << leftToRightRotationMatrix->cols << ", " << leftToRightRotationMatrix->rows << ")";
  }

  if (leftToRightTranslationVector->rows != 3 || leftToRightTranslationVector->cols != 1)
  {
    niftkNiftyCalThrow() << "Left to Right translation vector matrix should be 3x1, and its ("
                         << leftToRightTranslationVector->rows << ", " << leftToRightTranslationVector->cols << ")";
  }

  if (rvecsLeft->size() != tvecsLeft->size())
  {
    niftkNiftyCalThrow() << "Unequal extrinsic vectors: " << rvecsLeft->size()
                         << ", versus " << tvecsLeft->size();
  }

  m_RvecsLeft = rvecsLeft;
  m_TvecsLeft = tvecsLeft;
  m_LeftToRightRotationMatrix = leftToRightRotationMatrix;
  m_LeftToRightTranslationVector = leftToRightTranslationVector;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoIntrinsicsCalibrationCostFunction::SetDistortionParameters(cv::Mat* const leftDistortion,
                                                                               cv::Mat* const rightDistortion
                                                                              )
{
  if (leftDistortion == nullptr)
  {
    niftkNiftyCalThrow() << "Null left distortion parameters.";
  }

  if (rightDistortion == nullptr)
  {
    niftkNiftyCalThrow() << "Null right distortion parameters.";
  }

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

  cv::Mat rightIntrinsic = cv::Mat::zeros(3, 3, CV_64FC1);
  rightIntrinsic.at<double>(0, 0) = parameters[counter++];
  rightIntrinsic.at<double>(1, 1) = parameters[counter++];
  rightIntrinsic.at<double>(0, 2) = parameters[counter++];
  rightIntrinsic.at<double>(1, 2) = parameters[counter++];
  rightIntrinsic.at<double>(2, 2) = 1;

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
      *m_LeftDistortion,
      (*m_RvecsLeft)[numberOfViews],
      (*m_TvecsLeft)[numberOfViews],
      *m_LeftToRightRotationMatrix,
      *m_LeftToRightTranslationVector,
      rightIntrinsic,
      *m_RightDistortion,
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
