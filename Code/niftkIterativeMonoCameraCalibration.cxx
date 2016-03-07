/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIterativeMonoCameraCalibration.h"
#include "niftkMonoCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkHomographyUtilities.h"
#include "niftkPointUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
void ExtractTwoCopiesOfControlPoints(
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& list,
    std::list<PointSet>& a,
    std::list<PointSet>& b
    )
{
  a.clear();
  b.clear();

  std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::const_iterator iter;

  int counter = 0;
  for (iter = list.begin(); iter != list.end(); ++iter)
  {
    PointSet points = (*iter).first->GetPoints();
    if(points.empty())
    {
      niftkNiftyCalThrow() << "All input images should be valid calibration images containing extractable points, and " << counter << " isnt.";
    }

    a.push_back(points);
    b.push_back(points);
    counter++;
  }
}


//-----------------------------------------------------------------------------
void ExtractDistortedControlPoints(
    const std::pair< cv::Size2i, niftk::PointSet>& referenceData,
    const cv::Mat& intrinsic,
    const cv::Mat& distortion,
    const cv::Mat& originalImage,
    std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat>& outputDetectorAndImage,
    PointSet& outputPoints
    )
{
  cv::Mat undistortedImage;
  cv::Mat h;
  cv::Mat hInv;
  PointSet cp;
  PointSet cpi;
  PointSet cpid;

  // 1. Undistort and Unproject: Use the camera parameters to
  // undistort and unproject input images to a canonical pattern.
  cv::undistort(originalImage, undistortedImage, intrinsic, distortion, intrinsic);
  niftk::WarpImageByCorrespondingPoints(
        undistortedImage,
        intrinsic,                     // current estimate (updated each loop)
        distortion,                    // current estimate (updated each loop)
        outputPoints,                  // first time, its a copy of the original detected points,
                                       // after that, its updated, distorted points
        referenceData.second,          // specifies the proposed size of warped image
        referenceData.first,           // specifies the target point locations
        h,                             // output homography, written into
        outputDetectorAndImage.second  // output image, written into
        );

  // 2. Localize control points: Localize calibration pattern control
  // points in the canonical pattern.
  cp = outputDetectorAndImage.first->GetPoints();
  if(cp.empty())
  {
    niftkNiftyCalThrow() << "All warped images should still contain valid calibration images containing extractable points.";
  }

  // 3. Reproject: Project the control points using the estimated
  // camera parameters.
  hInv = h.inv(cv::DECOMP_SVD);
  niftk::WarpPointsByHomography(cp, hInv, cpi);
  niftk::DistortPoints(cpi, intrinsic, distortion, cpid);
  niftk::CopyPointsInto(cpid, outputPoints);
}


//-----------------------------------------------------------------------------
double IterativeMonoCameraCalibration(
    const Model3D& model,
    const std::pair< cv::Size2i, niftk::PointSet>& referenceImageData,
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndOriginalImages,
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndWarpedImages,
    cv::Size2i& imageSize,
    cv::Mat& intrinsic,
    cv::Mat& distortion,
    std::vector<cv::Mat>& rvecs,
    std::vector<cv::Mat>& tvecs,
    const int& cvFlags
    )
{
  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model should not be empty.";
  }
  if (detectorAndOriginalImages.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least two views of calibration points.";
  }
  if (detectorAndOriginalImages.size() != detectorAndWarpedImages.size())
  {
    niftkNiftyCalThrow() << "detectorAndOriginalImages must be the same size as detectorAndWarpedImages.";
  }
  if (referenceImageData.first.width == 0 || referenceImageData.first.height == 0)
  {
    niftkNiftyCalThrow() << "Invalid reference image size.";
  }
  if (referenceImageData.second.empty())
  {
    niftkNiftyCalThrow() << "Invalid reference image poinst.";
  }

  double projectedRMS = 0;

  // 1. Detect control points: Detect calibration pattern control
  // points (corners, circle or ring centers) in the input images.
  std::list<PointSet> pointsFromOriginalImages;
  std::list<PointSet> distortedPointsFromCanonicalImages;
  ExtractTwoCopiesOfControlPoints(detectorAndOriginalImages, pointsFromOriginalImages, distortedPointsFromCanonicalImages);

  // 2. Parameter Fitting: Use the detected control points to estimate
  // camera parameters using Levenberg-Marquardt.
  projectedRMS = niftk::MonoCameraCalibration(
        model,
        pointsFromOriginalImages,
        imageSize,
        intrinsic,
        distortion,
        rvecs,
        tvecs,
        cvFlags
        );

  std::cout << "Initial calibration, rms=" << projectedRMS << std::endl;
  std::cout << "Initial Fx=" << intrinsic.at<double>(0,0) << std::endl;
  std::cout << "Initial Fy=" << intrinsic.at<double>(1,1) << std::endl;
  std::cout << "Initial Cx=" << intrinsic.at<double>(0,2) << std::endl;
  std::cout << "Initial Cy=" << intrinsic.at<double>(1,2) << std::endl;
  std::cout << "Initial K1=" << distortion.at<double>(0,0) << std::endl;
  std::cout << "Initial K2=" << distortion.at<double>(0,1) << std::endl;
  std::cout << "Initial P1=" << distortion.at<double>(0,2) << std::endl;
  std::cout << "Initial P2=" << distortion.at<double>(0,3) << std::endl;

  int iterativeCvFlags = cvFlags | cv::CALIB_USE_INTRINSIC_GUESS;
  double previousRMS = std::numeric_limits<double>::max();
  unsigned int count = 0;

  do // until convergence
  {
    previousRMS = projectedRMS;

    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::const_iterator originalIter;
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::iterator canonicalIter;
    std::list<PointSet>::iterator pointsIter;

    for (originalIter = detectorAndOriginalImages.begin(),
         canonicalIter = detectorAndWarpedImages.begin(),
         pointsIter = distortedPointsFromCanonicalImages.begin();
         originalIter != detectorAndOriginalImages.end() &&
         canonicalIter != detectorAndWarpedImages.end() &&
         pointsIter != distortedPointsFromCanonicalImages.end();
         ++originalIter,
         ++canonicalIter,
         ++pointsIter
         )
    {
      niftk::ExtractDistortedControlPoints(
            referenceImageData,
            intrinsic,
            distortion,
            (*originalIter).second,
            (*canonicalIter),
            (*pointsIter)
            );
    }

    // 4. Parameter Fitting: Use the projected control points to refine
    // the camera parameters using Levenberg-Marquardt.
    projectedRMS = niftk::MonoCameraCalibration(
          model,
          distortedPointsFromCanonicalImages,
          imageSize,
          intrinsic,
          distortion,
          rvecs,
          tvecs,
          iterativeCvFlags
          );

    std::cout << "Iterative calibration iter=" << count++ << ", prms=" << previousRMS << ", rms=" << projectedRMS << std::endl;

  } while (projectedRMS < previousRMS &&
           fabs(projectedRMS - previousRMS) > 0.005);

  std::cout << "Final calibration, rms=" << projectedRMS << std::endl;
  std::cout << "Final Fx=" << intrinsic.at<double>(0,0) << std::endl;
  std::cout << "Final Fy=" << intrinsic.at<double>(1,1) << std::endl;
  std::cout << "Final Cx=" << intrinsic.at<double>(0,2) << std::endl;
  std::cout << "Final Cy=" << intrinsic.at<double>(1,2) << std::endl;
  std::cout << "Final K1=" << distortion.at<double>(0,0) << std::endl;
  std::cout << "Final K2=" << distortion.at<double>(0,1) << std::endl;
  std::cout << "Final P1=" << distortion.at<double>(0,2) << std::endl;
  std::cout << "Final P2=" << distortion.at<double>(0,3) << std::endl;

  return projectedRMS;
}

} // end namespace
