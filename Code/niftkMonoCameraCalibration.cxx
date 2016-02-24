/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkMonoCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-----------------------------------------------------------------------------
double MonoCameraCalibration(const Model3D& model,
                             const std::list<PointSet>& listOfPointSets,
                             const cv::Size2i& imageSize,
                             cv::Mat& intrinsic,
                             cv::Mat& distortion,
                             std::vector<cv::Mat>& rvecs,
                             std::vector<cv::Mat>& tvecs,
                             const bool& intrinsicsFixed
                             )
{
  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model should not be empty.";
  }
  if (listOfPointSets.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least two views of calibration points.";
  }

  double rms = 0;
  rvecs.clear();
  tvecs.clear();

  // Remember, each PointSet in listOfPointSets can have
  // a different number of points, and also each PointSet
  // can have different point IDs. Furthermore, assume
  // that point IDs in a given PointSet may not be valid
  // i.e. they are not contained in Model3D. So, we need
  // to extract only the ones that match in 2D and 3D.
  //
  // Fortunately, the OpenCV calibration can do this.
  // So we only have to convert the format.

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

    if (vectors3D.size() > 0 && vectors2D.size() > 0 && vectors3D.size() == vectors2D.size())
    {
      objectPoints.push_back(vectors3D);
      imagePoints.push_back(vectors2D);
      rvecs.push_back(cv::Mat());
      tvecs.push_back(cv::Mat());
    }
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

  if (!intrinsicsFixed)
  {
    cv::calibrateCamera(objectPoints,
                        imagePoints,
                        imageSize,
                        intrinsic,
                        distortion,
                        rvecs,
                        tvecs,
                        CV_CALIB_FIX_PRINCIPAL_POINT | CV_CALIB_FIX_ASPECT_RATIO
                        );

    cv::calibrateCamera(objectPoints,
                        imagePoints,
                        imageSize,
                        intrinsic,
                        distortion,
                        rvecs,
                        tvecs,
                        CV_CALIB_FIX_PRINCIPAL_POINT
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
    // Just do extrinsics.
    cv::solvePnP(objectPoints, imagePoints, intrinsic, distortion, rvecs, tvecs);
  }

  return rms;
}

} // end namespace
