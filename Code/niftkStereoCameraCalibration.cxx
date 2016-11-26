/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkStereoCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkMonoCameraCalibration.h"
#include "niftkPointUtilities.h"

#ifdef NIFTYCAL_WITH_ITK
#include <Internal/niftkNonLinearStereoIntrinsicsCalibrationOptimiser.h>
#include <Internal/niftkNonLinearStereoExtrinsicsCalibrationOptimiser.h>
#endif

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
    cv::Mat leftRot;
    cv::Rodrigues(rvecsLeft[i], leftRot);

    cv::Matx44d leftExtrinsic;
    cv::Matx44d leftToRight;

    leftExtrinsic = leftExtrinsic.eye();
    leftToRight = leftToRight.eye();

    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        leftExtrinsic(r, c) = leftRot.at<double>(r,c);
        leftToRight(r, c) = leftToRightRotationMatrix.at<double>(r, c);
      }
      leftExtrinsic(r, 3) = tvecsLeft[i].at<double>(0, r);
      leftToRight(r, 3) = leftToRightTranslationVector.at<double>(0, r);
    }

    cv::Matx44d rightExtrinsic = leftToRight * leftExtrinsic;
    cv::Matx33d rightRotation;

    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        rightRotation(r, c) = rightExtrinsic(r,c);
      }
    }

    cv::Mat rightRotationVec;
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
cv::Matx21d StereoCameraCalibration(const Model3D& model,
                                    const std::list<PointSet>& listOfLeftHandPointSets,
                                    const std::list<PointSet>& listOfRightHandPointSets,
                                    const cv::Size2i& imageSize,
                                    cv::Mat& intrinsicLeft,
                                    cv::Mat& distortionLeft,
                                    std::vector<cv::Mat>& rvecsLeft,
                                    std::vector<cv::Mat>& tvecsLeft,
                                    cv::Mat& intrinsicRight,
                                    cv::Mat& distortionRight,
                                    std::vector<cv::Mat>& rvecsRight,
                                    std::vector<cv::Mat>& tvecsRight,
                                    cv::Mat& leftToRightRotationMatrix,
                                    cv::Mat& leftToRightTranslationVector,
                                    cv::Mat& essentialMatrix,
                                    cv::Mat& fundamentalMatrix,
                                    const int& cvFlags,
                                    const bool& optimise3D
                                   )
{
  cv::Matx21d result;
  result(0, 0) = 0;
  result(1, 0) = 0;

  double projectedRMS = 0;
  double reconstructedRMS = 0;

  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model is empty.";
  }
  if (listOfLeftHandPointSets.size() < 1)
  {
    niftkNiftyCalThrow() << "Should have at least 1 view of calibration points for left camera.";
  }
  if (listOfRightHandPointSets.size() < 1)
  {
    niftkNiftyCalThrow() << "Should have at least 1 view of calibration points for right camera.";
  }
  if (listOfLeftHandPointSets.size() != listOfRightHandPointSets.size())
  {
    niftkNiftyCalThrow() << "Should have the same number of views in left and right channel.";
  }

  unsigned int viewCounter = 0;
  std::list<PointSet>::const_iterator iter;

  for (iter = listOfLeftHandPointSets.begin(); iter != listOfLeftHandPointSets.end(); ++iter)
  {
    if ((*iter).size() < 4)
    {
      niftkNiftyCalThrow() << "Should have 4 or more points in the " << viewCounter << "th left camera view.";
    }
    viewCounter++;
  }
  viewCounter = 0;
  for (iter = listOfRightHandPointSets.begin(); iter != listOfRightHandPointSets.end(); ++iter)
  {
    if ((*iter).size() < 4)
    {
      niftkNiftyCalThrow() << "Should have 4 or more points in the " << viewCounter << "th right camera view.";
    }
    viewCounter++;
  }

  std::vector<std::vector<cv::Vec3f> > objectPoints;
  std::vector<std::vector<cv::Vec2f> > leftImagePoints;
  std::vector<std::vector<cv::Vec2f> > rightImagePoints;

  // Remember at this point, the number of PointSets in left and right is the same.

  std::list<PointSet>::const_iterator leftListIter;
  std::list<PointSet>::const_iterator rightListIter;
  PointSet::const_iterator leftPointsIter;
  PointSet::const_iterator rightPointsIter;

  // Loop through each point set.
  viewCounter = 0;
  for (leftListIter = listOfLeftHandPointSets.begin(),
       rightListIter = listOfRightHandPointSets.begin();
       leftListIter != listOfLeftHandPointSets.end() &&
       rightListIter != listOfRightHandPointSets.end();
       ++leftListIter,
       ++rightListIter
       )
  {
    std::vector<cv::Vec3f> objectVectors3D;
    std::vector<cv::Vec2f> leftVectors2D;
    std::vector<cv::Vec2f> rightVectors2D;

    // A point must be visible in both left and right view for it to be added.
    for (leftPointsIter = (*leftListIter).begin(); leftPointsIter != (*leftListIter).end(); ++leftPointsIter)
    {
      niftk::NiftyCalIdType id = (*leftPointsIter).first;
      rightPointsIter = (*rightListIter).find(id);

      if (rightPointsIter != (*rightListIter).end())
      {
        // Found matching right hand point
        cv::Point3d p3 = (model.at((*leftPointsIter).first)).point;
        cv::Vec3f v3(p3.x, p3.y, p3.z);

        cv::Point2d p2l = ((*leftPointsIter).second).point;
        cv::Vec2f v2l(p2l.x, p2l.y);

        cv::Point2d p2r = ((*rightPointsIter).second).point;
        cv::Vec2f v2r(p2r.x, p2r.y);

        objectVectors3D.push_back(v3);
        leftVectors2D.push_back(v2l);
        rightVectors2D.push_back(v2r);
      }
    }

    if (objectVectors3D.size() >= 4 && leftVectors2D.size() >= 4 && rightVectors2D.size() >= 4
        && objectVectors3D.size() == leftVectors2D.size()
        && objectVectors3D.size() == rightVectors2D.size()
        )
    {
      objectPoints.push_back(objectVectors3D);
      leftImagePoints.push_back(leftVectors2D);
      rightImagePoints.push_back(rightVectors2D);
    }
    else
    {
      std::cout << "Warning: Dropping view " << viewCounter
                << ", as there were < 4 common points across both views." << std::endl;
    }
    viewCounter++;
  }

  // Sanity check
  if (objectPoints.size() == 0)
  {
    niftkNiftyCalThrow() << "No object points extracted.";
  }
  if (leftImagePoints.size() == 0)
  {
    niftkNiftyCalThrow() << "No left image points extracted.";
  }
  if (rightImagePoints.size() == 0)
  {
    niftkNiftyCalThrow() << "No right image points extracted.";
  }

  // Do calibration
  projectedRMS = cv::stereoCalibrate(objectPoints,
                                     leftImagePoints,
                                     rightImagePoints,
                                     intrinsicLeft,
                                     distortionLeft,
                                     intrinsicRight,
                                     distortionRight,
                                     imageSize,
                                     leftToRightRotationMatrix,
                                     leftToRightTranslationVector,
                                     essentialMatrix,
                                     fundamentalMatrix,
                                     cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 10000, 1e-10),
                                     cvFlags
                                    );

  niftk::ComputeStereoExtrinsics(rvecsLeft,
                                 tvecsRight,
                                 leftToRightRotationMatrix,
                                 leftToRightTranslationVector,
                                 rvecsRight,
                                 tvecsRight
                                );

  cv::Point3d rmsInEachAxis;
  reconstructedRMS = niftk::ComputeRMSReconstructionError(model,
                                                          listOfLeftHandPointSets,
                                                          listOfRightHandPointSets,
                                                          intrinsicLeft,
                                                          distortionLeft,
                                                          rvecsLeft,
                                                          tvecsLeft,
                                                          intrinsicRight,
                                                          distortionRight,
                                                          leftToRightRotationMatrix,
                                                          leftToRightTranslationVector,
                                                          rmsInEachAxis
                                                         );

#ifdef NIFTYCAL_WITH_ITK
  if (optimise3D)
  {
    Model3D* tmpModel = const_cast<Model3D*>(&model);

    // Now optimise RMS reconstruction error via intrinsics.
    niftk::NonLinearStereoIntrinsicsCalibrationOptimiser::Pointer intrinsicsOptimiser =
        niftk::NonLinearStereoIntrinsicsCalibrationOptimiser::New();

    intrinsicsOptimiser->SetModelAndPoints(tmpModel,
                                           &listOfLeftHandPointSets,
                                           &listOfRightHandPointSets
                                          );

    intrinsicsOptimiser->SetExtrinsics(&rvecsLeft,
                                       &tvecsLeft,
                                       &leftToRightRotationMatrix,
                                       &leftToRightTranslationVector);

    intrinsicsOptimiser->SetDistortionParameters(&distortionLeft, &distortionRight);

    intrinsicsOptimiser->Optimise(intrinsicLeft,
                                  intrinsicRight
                                 );

    // Now optimise RMS reconstruction error via extrinsics.
    niftk::NonLinearStereoExtrinsicsCalibrationOptimiser::Pointer extrinsicsOptimiser =
        niftk::NonLinearStereoExtrinsicsCalibrationOptimiser::New();

    extrinsicsOptimiser->SetModelAndPoints(tmpModel,
                                           &listOfLeftHandPointSets,
                                           &listOfRightHandPointSets);

    extrinsicsOptimiser->SetIntrinsics(&intrinsicLeft,
                                       &intrinsicRight
                                      );

    extrinsicsOptimiser->SetDistortionParameters(&distortionLeft, &distortionRight);

    reconstructedRMS = extrinsicsOptimiser->Optimise(rvecsLeft,
                                                     tvecsLeft,
                                                     leftToRightRotationMatrix,
                                                     leftToRightTranslationVector
                                                    );

    // Recompute re-projection error, as it will now be different.
    projectedRMS = niftk::ComputeRMSReprojectionError(model,
                                                      listOfLeftHandPointSets,
                                                      listOfRightHandPointSets,
                                                      intrinsicLeft,
                                                      distortionLeft,
                                                      rvecsLeft,
                                                      tvecsLeft,
                                                      intrinsicRight,
                                                      distortionRight,
                                                      leftToRightRotationMatrix,
                                                      leftToRightTranslationVector
                                                     );

    std::cout << "niftkStereoCameraCalibration:3D optimisation finished, rms2D=" << projectedRMS
              << ", rms3D=" << reconstructedRMS << std::endl;
  }
#endif

  result(0, 0) = projectedRMS;
  result(1, 0) = reconstructedRMS;

  return result;
}

} // end namespace
