/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkZhangCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkPointUtilities.h"
#include "niftkTsaiCameraCalibration.h"
#include <Internal/niftkCalibrationUtilities_p.h>

namespace niftk
{

//-----------------------------------------------------------------------------
double ZhangMonoCameraCalibration(const Model3D& model,
                                  const std::list<PointSet>& listOfPointSets,
                                  const cv::Size2i& imageSize,
                                  cv::Mat& intrinsic,
                                  cv::Mat& distortion,
                                  std::vector<cv::Mat>& rvecs,
                                  std::vector<cv::Mat>& tvecs,
                                  const int& cvFlags
                                 )
{
  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model is empty.";
  }
  if (listOfPointSets.empty())
  {
    niftkNiftyCalThrow() << "Should have at least 1 views of calibration points.";
  }

  double rms = 0;
  rvecs.clear();
  tvecs.clear();

  unsigned int viewCounter = 0;

  std::vector<std::vector<cv::Vec3f> > objectPoints;
  std::vector<std::vector<cv::Vec2f> > imagePoints;

  std::list<PointSet>::const_iterator listIter;
  PointSet::const_iterator pointsIter;

  for (listIter = listOfPointSets.begin(); listIter != listOfPointSets.end(); ++listIter)
  {
    std::vector<cv::Vec3f> vectors3D;
    std::vector<cv::Vec2f> vectors2D;

    for (pointsIter = (*listIter).begin(); pointsIter != (*listIter).end(); ++pointsIter)
    {
      // i.e. the 2D point ID is contained within the model.
      if (model.find((*pointsIter).first) != model.end())
      {
        cv::Point3d p3 = (model.at((*pointsIter).first)).point;
        cv::Vec3f v3(p3.x, p3.y, p3.z);

        cv::Point2d p2 = ((*pointsIter).second).point;
        cv::Vec2f v2(p2.x, p2.y);

        vectors3D.push_back(v3);
        vectors2D.push_back(v2);
      }
    }

    if (vectors3D.size() >= 4 && vectors2D.size() >= 4 && vectors3D.size() == vectors2D.size())
    {
      objectPoints.push_back(vectors3D);
      imagePoints.push_back(vectors2D);
      rvecs.push_back(cv::Mat::zeros(1, 3, CV_64FC1));
      tvecs.push_back(cv::Mat::zeros(1, 3, CV_64FC1));
    }
    else
    {
      std::cout << "Warning: Dropping view " << viewCounter << ", as there were < 4 points." << std::endl;
    }
    viewCounter++;
  }

  // Sanity check
  if (objectPoints.size() == 0)
  {
    niftkNiftyCalThrow() << "No object points extracted.";
  }
  if (imagePoints.size() == 0)
  {
    niftkNiftyCalThrow() << "No image points extracted.";
  }

  // Do calibration

  if (cvFlags == 0)
  {
    int flagsForFirstPass = CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_FIX_ASPECT_RATIO;

    if (!niftk::ModelIsPlanar(model))
    {
      // We run Tsais method on the first set of points
      // to initialise the intrinsics and distortion params.

      cv::Mat rvec;
      cv::Mat tvec;
      cv::Point2d sensorDims;
      sensorDims.x = 1;
      sensorDims.y = 1;

      niftk::TsaiMonoCameraCalibration(model,
        *(listOfPointSets.begin()),
        imageSize,
        sensorDims,
        intrinsic,
        distortion,
        rvec,
        tvec,
        true);

      flagsForFirstPass = CV_CALIB_USE_INTRINSIC_GUESS | flagsForFirstPass;
    }

    cv::calibrateCamera(objectPoints,
                        imagePoints,
                        imageSize,
                        intrinsic,
                        distortion,
                        rvecs,
                        tvecs,
                        flagsForFirstPass
                        );

    cv::calibrateCamera(objectPoints,
                        imagePoints,
                        imageSize,
                        intrinsic,
                        distortion,
                        rvecs,
                        tvecs,
                        CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_PRINCIPAL_POINT
                        );

    rms = cv::calibrateCamera(objectPoints,
                              imagePoints,
                              imageSize,
                              intrinsic,
                              distortion,
                              rvecs,
                              tvecs,
                              CV_CALIB_USE_INTRINSIC_GUESS
                              );
  }
  else
  {
    rms = cv::calibrateCamera(objectPoints,
                              imagePoints,
                              imageSize,
                              intrinsic,
                              distortion,
                              rvecs,
                              tvecs,
                              cvFlags
                              );
  }

  return rms;
}

} // end namespace
