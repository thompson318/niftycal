/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkCalibrationUtilities_p.h"
#include <niftkPointUtilities.h>
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>

namespace niftk
{

//-----------------------------------------------------------------------------
int Signum(const double& x)
{
  if (x < 0)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}


//-----------------------------------------------------------------------------
void ComputeStereoExtrinsics(const std::vector<cv::Mat>& rvecsLeft,
                             const std::vector<cv::Mat>& tvecsLeft,
                             const cv::Mat& leftToRightRotationMatrix,
                             const cv::Mat& leftToRightTranslationVector,
                             std::vector<cv::Mat>& rvecsRight,
                             std::vector<cv::Mat>& tvecsRight
                            )
{
  // First make sure we have the right number of rvecsRight and rvecsLeft allocated.
  rvecsRight.clear();
  tvecsRight.clear();
  for (int i = 0; i < rvecsLeft.size(); i++)
  {
    rvecsRight.push_back(cvCreateMat(1, 3, CV_64FC1));
    tvecsRight.push_back(cvCreateMat(1, 3, CV_64FC1));
  }

  // Then make sure rvecs and tvecs are consistent left and right.
  for (int i = 0; i < rvecsLeft.size(); i++)
  {
    cv::Mat leftRot = cvCreateMat(3, 3, CV_64FC1);
    cv::Rodrigues(rvecsLeft[i], leftRot);

    cv::Matx44d leftExtrinsic = cv::Matx44d::eye();
    cv::Matx44d leftToRight = cv::Matx44d::eye();

    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        leftExtrinsic(r, c) = leftRot.at<double>(r, c);
        leftToRight(r, c) = leftToRightRotationMatrix.at<double>(r, c);
      }
      leftExtrinsic(r, 3) = tvecsLeft[i].at<double>(0, r);
      leftToRight(r, 3) = leftToRightTranslationVector.at<double>(r, 0);
    }

    cv::Matx44d rightExtrinsic = leftToRight * leftExtrinsic;
    cv::Mat rightRotation = cvCreateMat(3, 3, CV_64FC1);

    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        rightRotation.at<double>(r, c) = rightExtrinsic(r, c);
      }
    }

    cv::Mat rightRotationVec = cvCreateMat(1, 3, CV_64FC1);
    niftk::SafeRodrigues(rightRotation, rightRotationVec);

    rvecsRight[i].at<double>(0, 0) = rightRotationVec.at<double>(0,0);
    rvecsRight[i].at<double>(0, 1) = rightRotationVec.at<double>(0,1);
    rvecsRight[i].at<double>(0, 2) = rightRotationVec.at<double>(0,2);

    tvecsRight[i].at<double>(0, 0) = rightExtrinsic(0,3);
    tvecsRight[i].at<double>(0, 1) = rightExtrinsic(1,3);
    tvecsRight[i].at<double>(0, 2) = rightExtrinsic(2,3);
  }
}


//-----------------------------------------------------------------------------
void ComputeMonoProjectionErrors(const niftk::Model3D& model,
                                 const niftk::PointSet& points,
                                 const cv::Matx44d& extrinsic,
                                 const cv::Mat& intrinsic,
                                 const cv::Mat& distortion,
                                 itk::MultipleValuedCostFunction::MeasureType& errorValues
                                )
{
  std::vector<cv::Point2f> observed;
  std::vector<cv::Point2f> projected;
  std::vector<niftk::NiftyCalIdType> ids;

  niftk::ProjectMatchingPoints(model,
                               points,
                               extrinsic,
                               intrinsic,
                               distortion,
                               observed,
                               projected,
                               ids
                              );

  unsigned long int totalPointCounter = 0;
  unsigned long int numberOfValues = observed.size() * 2;

  errorValues.clear();
  errorValues.SetSize(numberOfValues);

  for (unsigned int i = 0; i < observed.size(); i++)
  {
    errorValues[totalPointCounter++] = (observed[i].x - projected[i].x);
    errorValues[totalPointCounter++] = (observed[i].y - projected[i].y);
  }
}


//-----------------------------------------------------------------------------
void ComputeMonoProjectionErrors(const Model3D* const model,
                                 const std::list<PointSet>* const points,
                                 const itk::MultipleValuedCostFunction::ParametersType& parameters,
                                 itk::MultipleValuedCostFunction::MeasureType& errors
                                )
{
  if (parameters.size() < 15)
  {
    niftkNiftyCalThrow() << "Too few parameters, must be at least 15";
  }
  if ((parameters.size() - 9) % 6 != 0)
  {
    niftkNiftyCalThrow() << "Incorrect number of parameters, must be at least intrinsic (4DOF), distortion (5DOF), then Nx6DOF.";
  }
  if ((parameters.size() - 9) / 6 != points->size())
  {
    niftkNiftyCalThrow() << "Incorrect number of parameters, the number of sets of 6DOF extrinsic parameters, must match the number of views";
  }

  unsigned int parameterCounter = 0;

  cv::Mat intrinsic = cv::Mat::eye(3, 3, CV_64FC1);
  intrinsic.at<double>(0, 0) = parameters[parameterCounter++];
  intrinsic.at<double>(1, 1) = parameters[parameterCounter++];
  intrinsic.at<double>(0, 2) = parameters[parameterCounter++];
  intrinsic.at<double>(1, 2) = parameters[parameterCounter++];

  cv::Mat distortion = cvCreateMat(1, 5, CV_64FC1);
  distortion.at<double>(0, 0) = parameters[parameterCounter++];
  distortion.at<double>(0, 1) = parameters[parameterCounter++];
  distortion.at<double>(0, 2) = parameters[parameterCounter++];
  distortion.at<double>(0, 3) = parameters[parameterCounter++];
  distortion.at<double>(0, 4) = parameters[parameterCounter++];

  itk::MultipleValuedCostFunction::MeasureType errorsPerView;
  unsigned long int errorCounter = 0;

  std::list<PointSet>::const_iterator iter;
  for (iter = points->begin(); iter != points->end(); iter++)
  {
    cv::Mat rvec = cvCreateMat(1, 3, CV_64FC1);
    rvec.at<double>(0, 0) = parameters[parameterCounter++];
    rvec.at<double>(0, 1) = parameters[parameterCounter++];
    rvec.at<double>(0, 2) = parameters[parameterCounter++];

    cv::Mat tvec = cvCreateMat(1, 3, CV_64FC1);
    tvec.at<double>(0, 0) = parameters[parameterCounter++];
    tvec.at<double>(0, 1) = parameters[parameterCounter++];
    tvec.at<double>(0, 2) = parameters[parameterCounter++];

    cv::Matx44d extrinsic = niftk::RodriguesToMatrix(rvec, tvec);

    niftk::ComputeMonoProjectionErrors(*model, *iter, extrinsic, intrinsic, distortion, errorsPerView);

    for (unsigned long int i = 0; i < errorsPerView.size(); i++)
    {
      errors[errorCounter++] = errorsPerView[i];
    }
  }
}


//-----------------------------------------------------------------------------
void ComputeStereoProjectionErrors(const Model3D& model,
                                   const PointSet& leftPoints,
                                   const PointSet& rightPoints,
                                   const cv::Matx44d& leftExtrinsic,
                                   const cv::Mat& leftIntrinsic,
                                   const cv::Mat& leftDistortion,
                                   const cv::Matx44d& rightExtrinsic,
                                   const cv::Mat& rightIntrinsic,
                                   const cv::Mat& rightDistortion,
                                   itk::MultipleValuedCostFunction::MeasureType& errors
                                  )
{
  itk::MultipleValuedCostFunction::MeasureType leftErrors;
  ComputeMonoProjectionErrors(model, leftPoints, leftExtrinsic, leftIntrinsic, leftDistortion, leftErrors);

  itk::MultipleValuedCostFunction::MeasureType rightErrors;
  ComputeMonoProjectionErrors(model, rightPoints, rightExtrinsic, rightIntrinsic, rightDistortion, rightErrors);

  errors.clear();
  errors.SetSize(leftErrors.size() + rightErrors.size());

  unsigned long int errorCounter = 0;
  for (unsigned long int i = 0; i < leftErrors.size(); i++)
  {
    errors[errorCounter++] = leftErrors[i];
  }

  for (unsigned long int i = 0; i < rightErrors.size(); i++)
  {
    errors[errorCounter++] = rightErrors[i];
  }
}


//-----------------------------------------------------------------------------
void ComputeStereoProjectionErrors(const Model3D* const model,
                                   const std::list<PointSet>* const leftPoints,
                                   const std::list<PointSet>* const rightPoints,
                                   const itk::MultipleValuedCostFunction::ParametersType& parameters,
                                   itk::MultipleValuedCostFunction::MeasureType& errors
                                  )
{
  if (parameters.size() < 30)
  {
    niftkNiftyCalThrow() << "Too few parameters, must be at least 30";
  }
  if ((parameters.size() - 30) % 6 != 0)
  {
    niftkNiftyCalThrow() << "Incorrect number of parameters, must be at least intrinsic (4DOF), distortion (5DOF) for both left and right, then [6DOF stereo extrinsic], then [Nx6DOF].";
  }
  if (leftPoints->size() != rightPoints->size())
  {
    niftkNiftyCalThrow() << "The number of left point sets:" << leftPoints->size()
                         << ", doesn't match the number of right point sets:" << rightPoints->size();
  }

  unsigned int parameterCounter = 0;

  cv::Mat leftIntrinsic = cv::Mat::eye(3, 3, CV_64FC1);
  leftIntrinsic.at<double>(0, 0) = parameters[parameterCounter++];
  leftIntrinsic.at<double>(1, 1) = parameters[parameterCounter++];
  leftIntrinsic.at<double>(0, 2) = parameters[parameterCounter++];
  leftIntrinsic.at<double>(1, 2) = parameters[parameterCounter++];

  cv::Mat leftDistortion = cvCreateMat(1, 5, CV_64FC1);
  leftDistortion.at<double>(0, 0) = parameters[parameterCounter++];
  leftDistortion.at<double>(0, 1) = parameters[parameterCounter++];
  leftDistortion.at<double>(0, 2) = parameters[parameterCounter++];
  leftDistortion.at<double>(0, 3) = parameters[parameterCounter++];
  leftDistortion.at<double>(0, 4) = parameters[parameterCounter++];

  cv::Mat rightIntrinsic = cv::Mat::eye(3, 3, CV_64FC1);
  rightIntrinsic.at<double>(0, 0) = parameters[parameterCounter++];
  rightIntrinsic.at<double>(1, 1) = parameters[parameterCounter++];
  rightIntrinsic.at<double>(0, 2) = parameters[parameterCounter++];
  rightIntrinsic.at<double>(1, 2) = parameters[parameterCounter++];

  cv::Mat rightDistortion = cvCreateMat(1, 5, CV_64FC1);
  rightDistortion.at<double>(0, 0) = parameters[parameterCounter++];
  rightDistortion.at<double>(0, 1) = parameters[parameterCounter++];
  rightDistortion.at<double>(0, 2) = parameters[parameterCounter++];
  rightDistortion.at<double>(0, 3) = parameters[parameterCounter++];
  rightDistortion.at<double>(0, 4) = parameters[parameterCounter++];

  cv::Mat leftToRightRotationVector = cvCreateMat(1, 3, CV_64FC1);
  leftToRightRotationVector.at<double>(0, 0) = parameters[parameterCounter++];
  leftToRightRotationVector.at<double>(0, 1) = parameters[parameterCounter++];
  leftToRightRotationVector.at<double>(0, 2) = parameters[parameterCounter++];

  cv::Mat leftToRightTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  leftToRightTranslationVector.at<double>(0, 0) = parameters[parameterCounter++];
  leftToRightTranslationVector.at<double>(0, 1) = parameters[parameterCounter++];
  leftToRightTranslationVector.at<double>(0, 2) = parameters[parameterCounter++];

  cv::Matx44d leftToRightExtrinsic = niftk::RodriguesToMatrix(leftToRightRotationVector, leftToRightTranslationVector);

  itk::MultipleValuedCostFunction::MeasureType errorsPerView;
  unsigned long int errorCounter = 0;

  std::list<PointSet>::const_iterator leftIter;
  std::list<PointSet>::const_iterator rightIter;
  for (leftIter = leftPoints->begin(), rightIter = rightPoints->begin();
       leftIter != leftPoints->end();
       leftIter++, rightIter++
      )
  {
    cv::Mat rvec = cvCreateMat(1, 3, CV_64FC1);
    rvec.at<double>(0, 0) = parameters[parameterCounter++];
    rvec.at<double>(0, 1) = parameters[parameterCounter++];
    rvec.at<double>(0, 2) = parameters[parameterCounter++];

    cv::Mat tvec = cvCreateMat(1, 3, CV_64FC1);
    tvec.at<double>(0, 0) = parameters[parameterCounter++];
    tvec.at<double>(0, 1) = parameters[parameterCounter++];
    tvec.at<double>(0, 2) = parameters[parameterCounter++];

    cv::Matx44d leftExtrinsic = niftk::RodriguesToMatrix(rvec, tvec);
    cv::Matx44d rightExtrinsic = leftToRightExtrinsic * leftExtrinsic;

    niftk::ComputeStereoProjectionErrors(*model, *leftIter, *rightIter, leftExtrinsic, leftIntrinsic, leftDistortion, rightExtrinsic, rightIntrinsic, rightDistortion, errorsPerView);
    for (unsigned long int i = 0; i < errorsPerView.size(); i++)
    {
      errors[errorCounter++] = errorsPerView[i];
    }
  }
}


//-----------------------------------------------------------------------------
void ComputeStereoReconstructionErrors(const Model3D& model,
                                       const PointSet& leftPoints,
                                       const PointSet& rightPoints,
                                       const cv::Matx44d& leftExtrinsic,
                                       const cv::Mat& leftIntrinsic,
                                       const cv::Mat& leftDistortion,
                                       const cv::Mat& leftToRightRotationMatrix,
                                       const cv::Mat& leftToRightTranslationVector,
                                       const cv::Mat& rightIntrinsic,
                                       const cv::Mat& rightDistortion,
                                       itk::MultipleValuedCostFunction::MeasureType& errorValues
                                      )
{
  niftk::Model3D triangulatedModelInLeftCameraSpace;

  cv::Mat rvecLeft = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat tvecLeft = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(leftExtrinsic, rvecLeft, tvecLeft);

  niftk::TriangulatePointPairs(
    leftPoints,
    rightPoints,
    leftIntrinsic,
    leftDistortion,
    rvecLeft,
    tvecLeft,
    leftToRightRotationMatrix,
    leftToRightTranslationVector,
    rightIntrinsic,
    rightDistortion,
    triangulatedModelInLeftCameraSpace
  );

  cv::Matx44d modelToCamera = leftExtrinsic;
  cv::Matx44d cameraToModel = modelToCamera.inv(cv::DECOMP_SVD);

  niftk::Model3D triangulatedModelInModelSpace = niftk::TransformModel(triangulatedModelInLeftCameraSpace,
                                                                       cameraToModel);

  std::list<niftk::PointSet> lp;
  lp.push_back(leftPoints);

  std::list<niftk::PointSet> rp;
  rp.push_back(rightPoints);

  unsigned long int totalPointCounter = 0;
  unsigned long int numberOfValues = (niftk::GetNumberOfTriangulatablePoints(model, lp, rp)); // * 3;

  errorValues.clear();
  errorValues.SetSize(numberOfValues);

  niftk::Model3D::const_iterator modelIter;
  for (modelIter = triangulatedModelInModelSpace.begin();
       modelIter != triangulatedModelInModelSpace.end();
       ++modelIter
       )
  {
    niftk::NiftyCalIdType id = (*modelIter).first;
    Model3D::const_iterator goldIter = model.find(id);
    if (goldIter == model.end())
    {
      niftkNiftyCalThrow() << "Failed to find point " << id << " in gold standard model.";
    }
    niftk::Point3D tp = (*modelIter).second;
    niftk::Point3D gsp = (*goldIter).second;
    errorValues[totalPointCounter++] = std::sqrt(
        (tp.point.x - gsp.point.x) * (tp.point.x - gsp.point.x)
      + (tp.point.y - gsp.point.y) * (tp.point.y - gsp.point.y)
      + (tp.point.z - gsp.point.z) * (tp.point.z - gsp.point.z)
      );
  }
}


//-----------------------------------------------------------------------------
void ComputeStereoReconstructionErrors(const Model3D* const model,
                                       const std::list<PointSet>* const leftPoints,
                                       const std::list<PointSet>* const rightPoints,
                                       const itk::MultipleValuedCostFunction::ParametersType& parameters,
                                       itk::MultipleValuedCostFunction::MeasureType& errors
                                      )
{
  if (parameters.size() < 30)
  {
    niftkNiftyCalThrow() << "Too few parameters, must be at least 30";
  }
  if ((parameters.size() - 30) % 6 != 0)
  {
    niftkNiftyCalThrow() << "Incorrect number of parameters, must be at least intrinsic (4DOF), distortion (5DOF) for both left and right, then 6DOF stereo extrinsic, then Nx6DOF.";
  }
  if ((parameters.size() - 24) / 6 != leftPoints->size())
  {
    niftkNiftyCalThrow() << "Incorrect number of parameters, the number of sets of 6DOF extrinsic parameters, must match the number of views";
  }
  if (leftPoints->size() != rightPoints->size())
  {
    niftkNiftyCalThrow() << "The number of left point sets:" << leftPoints->size()
                         << ", doesn't match the number of right point sets:" << rightPoints->size();
  }

  unsigned int parameterCounter = 0;

  cv::Mat leftIntrinsic = cv::Mat::eye(3, 3, CV_64FC1);
  leftIntrinsic.at<double>(0, 0) = parameters[parameterCounter++];
  leftIntrinsic.at<double>(1, 1) = parameters[parameterCounter++];
  leftIntrinsic.at<double>(0, 2) = parameters[parameterCounter++];
  leftIntrinsic.at<double>(1, 2) = parameters[parameterCounter++];

  cv::Mat leftDistortion = cvCreateMat(1, 5, CV_64FC1);
  leftDistortion.at<double>(0, 0) = parameters[parameterCounter++];
  leftDistortion.at<double>(0, 1) = parameters[parameterCounter++];
  leftDistortion.at<double>(0, 2) = parameters[parameterCounter++];
  leftDistortion.at<double>(0, 3) = parameters[parameterCounter++];
  leftDistortion.at<double>(0, 4) = parameters[parameterCounter++];

  cv::Mat rightIntrinsic = cv::Mat::eye(3, 3, CV_64FC1);
  rightIntrinsic.at<double>(0, 0) = parameters[parameterCounter++];
  rightIntrinsic.at<double>(1, 1) = parameters[parameterCounter++];
  rightIntrinsic.at<double>(0, 2) = parameters[parameterCounter++];
  rightIntrinsic.at<double>(1, 2) = parameters[parameterCounter++];

  cv::Mat rightDistortion = cvCreateMat(1, 5, CV_64FC1);
  rightDistortion.at<double>(0, 0) = parameters[parameterCounter++];
  rightDistortion.at<double>(0, 1) = parameters[parameterCounter++];
  rightDistortion.at<double>(0, 2) = parameters[parameterCounter++];
  rightDistortion.at<double>(0, 3) = parameters[parameterCounter++];
  rightDistortion.at<double>(0, 4) = parameters[parameterCounter++];

  cv::Mat leftToRightRotationVector = cvCreateMat(1, 3, CV_64FC1);
  leftToRightRotationVector.at<double>(0, 0) = parameters[parameterCounter++];
  leftToRightRotationVector.at<double>(0, 1) = parameters[parameterCounter++];
  leftToRightRotationVector.at<double>(0, 2) = parameters[parameterCounter++];

  cv::Mat leftToRightTranslationVector = cvCreateMat(3, 1, CV_64FC1);
  leftToRightTranslationVector.at<double>(0, 0) = parameters[parameterCounter++];
  leftToRightTranslationVector.at<double>(1, 0) = parameters[parameterCounter++];
  leftToRightTranslationVector.at<double>(2, 0) = parameters[parameterCounter++];

  cv::Mat leftToRightRotationMatrix = cvCreateMat(3, 3, CV_64FC1);
  cv::Rodrigues(leftToRightRotationVector, leftToRightRotationMatrix);

  itk::MultipleValuedCostFunction::MeasureType errorsPerView;
  unsigned long int errorCounter = 0;

  std::list<PointSet>::const_iterator leftIter;
  std::list<PointSet>::const_iterator rightIter;
  for (leftIter = leftPoints->begin(), rightIter = rightPoints->begin();
       leftIter != leftPoints->end();
       leftIter++, rightIter++
      )
  {
    cv::Mat rvec = cvCreateMat(1, 3, CV_64FC1);
    rvec.at<double>(0, 0) = parameters[parameterCounter++];
    rvec.at<double>(0, 1) = parameters[parameterCounter++];
    rvec.at<double>(0, 2) = parameters[parameterCounter++];

    cv::Mat tvec = cvCreateMat(1, 3, CV_64FC1);
    tvec.at<double>(0, 0) = parameters[parameterCounter++];
    tvec.at<double>(0, 1) = parameters[parameterCounter++];
    tvec.at<double>(0, 2) = parameters[parameterCounter++];

    cv::Matx44d leftExtrinsic = niftk::RodriguesToMatrix(rvec, tvec);

    niftk::ComputeStereoReconstructionErrors(*model,
                                             *leftIter,
                                             *rightIter,
                                             leftExtrinsic,
                                             leftIntrinsic,
                                             leftDistortion,
                                             leftToRightRotationMatrix,
                                             leftToRightTranslationVector,
                                             rightIntrinsic,
                                             rightDistortion,
                                             errorsPerView
                                             );

    for (unsigned long int i = 0; i < errorsPerView.size(); i++)
    {
      errors[errorCounter++] = errorsPerView[i];
    }
  }
}

} // end namespace
