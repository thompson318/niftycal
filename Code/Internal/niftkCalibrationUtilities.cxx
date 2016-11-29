/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkCalibrationUtilities_p.h"

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

} // end namespace
