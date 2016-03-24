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

namespace niftk
{

//-----------------------------------------------------------------------------
double StereoCameraCalibration(const Model3D& model,
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
                               cv::Mat& left2RightRotation,
                               cv::Mat& left2RightTranslation,
                               cv::Mat& essentialMatrix,
                               cv::Mat& fundamentalMatrix,
                               const int& cvFlags
                              )
{
  double rms = 0;
  rvecsLeft.clear();
  tvecsLeft.clear();
  rvecsRight.clear();
  tvecsRight.clear();

  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model is empty.";
  }
  if (listOfLeftHandPointSets.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least 2 views of calibration points for left camera.";
  }
  if (listOfRightHandPointSets.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least 2 views of calibration points for right camera.";
  }
  if (listOfLeftHandPointSets.size() != listOfRightHandPointSets.size())
  {
    niftkNiftyCalThrow() << "Should have the same number of views in left and right channel.";
  }

  std::list<PointSet>::const_iterator iter;
  int counter = 0;
  for (iter = listOfLeftHandPointSets.begin(); iter != listOfLeftHandPointSets.end(); ++iter)
  {
    counter++;
    if ((*iter).size() == 0)
    {
      niftkNiftyCalThrow() << "Should have 1 or more points in the " << counter << "th left camera view.";
    }
  }
  counter = 0;
  for (iter = listOfRightHandPointSets.begin(); iter != listOfRightHandPointSets.end(); ++iter)
  {
    counter++;
    if ((*iter).size() == 0)
    {
      niftkNiftyCalThrow() << "Should have 1 or more points in the " << counter << "th right camera view.";
    }
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
      niftk::IdType id = (*leftPointsIter).first;
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

    if (objectVectors3D.size() > 0 && leftVectors2D.size() > 0 && rightVectors2D.size() > 0
        && objectVectors3D.size() == leftVectors2D.size()
        && objectVectors3D.size() == rightVectors2D.size()
        )
    {
      objectPoints.push_back(objectVectors3D);
      leftImagePoints.push_back(leftVectors2D);
      rightImagePoints.push_back(rightVectors2D);
      rvecsLeft.push_back(cv::Mat());
      tvecsLeft.push_back(cv::Mat());
      rvecsRight.push_back(cv::Mat());
      tvecsRight.push_back(cv::Mat());
    }
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
  rms = cv::stereoCalibrate(objectPoints,
                            leftImagePoints,
                            rightImagePoints,
                            intrinsicLeft,
                            distortionLeft,
                            intrinsicRight,
                            distortionRight,
                            imageSize,
                            left2RightRotation,
                            left2RightTranslation,
                            essentialMatrix,
                            fundamentalMatrix,
                            cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-10),
                            cvFlags
                            );

  // Additionally make sure rvecs and tvecs are consistent left and right.

  // 1. Solve left.
  cv::calibrateCamera(objectPoints,
                      leftImagePoints,
                      imageSize,
                      intrinsicLeft,
                      distortionLeft,
                      rvecsLeft,
                      tvecsLeft,
                      cvFlags | CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_INTRINSIC
                      );

  // 2. Solve Right. (not really needed, but useful for debugging).
  cv::calibrateCamera(objectPoints,
                      rightImagePoints,
                      imageSize,
                      intrinsicRight,
                      distortionRight,
                      rvecsRight,
                      tvecsRight,
                      cvFlags | CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_INTRINSIC
                      );

  // 3. Make consistent rvecs, tvecs, left and right,
  // by creating the right extrinsics, by composing
  // the left extrinsics, and the left-to-right transform.
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
        leftToRight(r, c) = left2RightRotation.at<double>(r, c);
      }
      leftExtrinsic(r, 3) = tvecsLeft[i].at<double>(0, r);
      leftToRight(r, 3) = left2RightTranslation.at<double>(0, r);
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

  return rms;
}

} // end namespace
