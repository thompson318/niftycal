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
    cv::Matx33d rightRotation;

    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        rightRotation(r, c) = rightExtrinsic(r, c);
      }
    }

    cv::Mat rightRotationVec = cvCreateMat(1, 3, CV_64FC1);
    cv::Rodrigues(rightRotation, rightRotationVec);

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
    niftkNiftyCalThrow() << "Incorrect number of parameters, must be at least intrinsic (4DOF), distortion (5DOF), then 6N DOF.";
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
  unsigned long int valueCounter = 0;

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
      errors[valueCounter++] = errorsPerView[i];
    }
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

}


//-----------------------------------------------------------------------------
void ComputeStereoReconstructionErrors(const Model3D* const model,
                                       const std::list<PointSet>* const leftPoints,
                                       const std::list<PointSet>* const rightPoints,
                                       const itk::MultipleValuedCostFunction::ParametersType& parameters,
                                       itk::MultipleValuedCostFunction::MeasureType& errors
                                      )
{

}

} // end namespace
