/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIterativeStereoCameraCalibration.h"
#include "niftkIterativeMonoCameraCalibration.h"
#include "niftkMonoCameraCalibration.h"
#include "niftkStereoCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkHomographyUtilities.h"
#include "niftkPointUtilities.h"
#include <highgui.h>

namespace niftk
{

//-----------------------------------------------------------------------------
double IterativeStereoCameraCalibration(
    const Model3D& model,
    const std::pair< cv::Size2i, niftk::PointSet>& referenceImageData,
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndOriginalImagesLeft,
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndOriginalImagesRight,
    const cv::Size2i& imageSize,
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndWarpedImagesLeft,
    cv::Mat& intrinsicLeft,
    cv::Mat& distortionLeft,
    std::vector<cv::Mat>& rvecsLeft,
    std::vector<cv::Mat>& tvecsLeft,
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndWarpedImagesRight,
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
  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model should not be empty.";
  }
  if (detectorAndOriginalImagesLeft.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least two left-hand views of calibration points.";
  }
  if (detectorAndOriginalImagesRight.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least two right-hand views of calibration points.";
  }

  if (detectorAndOriginalImagesLeft.size() != detectorAndOriginalImagesRight.size()
      || detectorAndOriginalImagesLeft.size() != detectorAndWarpedImagesLeft.size()
      || detectorAndOriginalImagesLeft.size() != detectorAndWarpedImagesRight.size()
      || detectorAndOriginalImagesRight.size() != detectorAndWarpedImagesLeft.size()
      || detectorAndOriginalImagesRight.size() != detectorAndWarpedImagesRight.size()
      || detectorAndWarpedImagesLeft.size() != detectorAndWarpedImagesRight.size()
      )
  {
    niftkNiftyCalThrow() << "Inconsistent number of images and detector pairs.";
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

  std::list<PointSet> pointsFromOriginalImagesLeft;
  std::list<PointSet> distortedPointsFromCanonicalImagesLeft;
  ExtractTwoCopiesOfControlPoints(detectorAndOriginalImagesLeft, pointsFromOriginalImagesLeft, distortedPointsFromCanonicalImagesLeft);

  std::list<PointSet> pointsFromOriginalImagesRight;
  std::list<PointSet> distortedPointsFromCanonicalImagesRight;
  ExtractTwoCopiesOfControlPoints(detectorAndOriginalImagesRight, pointsFromOriginalImagesRight, distortedPointsFromCanonicalImagesRight);

  // 2. Parameter Fitting: Use the detected control points to estimate
  // camera parameters using Levenberg-Marquardt.
  niftk::MonoCameraCalibration(
        model,
        pointsFromOriginalImagesLeft,
        imageSize,
        intrinsicLeft,
        distortionLeft,
        rvecsLeft,
        tvecsLeft,
        cvFlags
        );

  niftk::MonoCameraCalibration(
        model,
        pointsFromOriginalImagesRight,
        imageSize,
        intrinsicRight,
        distortionRight,
        rvecsRight,
        tvecsRight,
        cvFlags
        );

  projectedRMS = niftk::StereoCameraCalibration(
        model,
        pointsFromOriginalImagesLeft,
        pointsFromOriginalImagesRight,
        imageSize,
        intrinsicLeft,
        distortionLeft,
        rvecsLeft,
        tvecsLeft,
        intrinsicRight,
        distortionRight,
        rvecsRight,
        tvecsRight,
        left2RightRotation,
        left2RightTranslation,
        essentialMatrix,
        fundamentalMatrix
        );

  std::cout << "Initial stereo calibration, rms=" << projectedRMS << std::endl;
  std::cout << "Initial Fxl=" << intrinsicLeft.at<double>(0,0) << std::endl;
  std::cout << "Initial Fyl=" << intrinsicLeft.at<double>(1,1) << std::endl;
  std::cout << "Initial Cxl=" << intrinsicLeft.at<double>(0,2) << std::endl;
  std::cout << "Initial Cyl=" << intrinsicLeft.at<double>(1,2) << std::endl;
  std::cout << "Initial K1l=" << distortionLeft.at<double>(0,0) << std::endl;
  std::cout << "Initial K2l=" << distortionLeft.at<double>(0,1) << std::endl;
  std::cout << "Initial P1l=" << distortionLeft.at<double>(0,2) << std::endl;
  std::cout << "Initial P2l=" << distortionLeft.at<double>(0,3) << std::endl;
  std::cout << "Initial Fxr=" << intrinsicRight.at<double>(0,0) << std::endl;
  std::cout << "Initial Fyr=" << intrinsicRight.at<double>(1,1) << std::endl;
  std::cout << "Initial Cxr=" << intrinsicRight.at<double>(0,2) << std::endl;
  std::cout << "Initial Cyr=" << intrinsicRight.at<double>(1,2) << std::endl;
  std::cout << "Initial K1r=" << intrinsicRight.at<double>(0,0) << std::endl;
  std::cout << "Initial K2r=" << intrinsicRight.at<double>(0,1) << std::endl;
  std::cout << "Initial P1r=" << intrinsicRight.at<double>(0,2) << std::endl;
  std::cout << "Initial P2r=" << intrinsicRight.at<double>(0,3) << std::endl;

  int iterativeCvFlags = cvFlags | cv::CALIB_USE_INTRINSIC_GUESS;
  double previousRMS = std::numeric_limits<double>::max();
  unsigned int count = 0;

  do // until convergence
  {
    previousRMS = projectedRMS;

    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::const_iterator originalIter;
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::iterator canonicalIter;
    std::list<PointSet>::iterator pointsIter;

    // Do all left.
    for (originalIter = detectorAndOriginalImagesLeft.begin(),
         canonicalIter = detectorAndWarpedImagesLeft.begin(),
         pointsIter = distortedPointsFromCanonicalImagesLeft.begin();
         originalIter != detectorAndOriginalImagesLeft.end() &&
         canonicalIter != detectorAndWarpedImagesLeft.end() &&
         pointsIter != distortedPointsFromCanonicalImagesLeft.end();
         ++originalIter,
         ++canonicalIter,
         ++pointsIter
         )
    {
      niftk::ExtractDistortedControlPoints(
            referenceImageData,
            intrinsicLeft,
            distortionLeft,
            (*originalIter).second,
            (*canonicalIter),
            (*pointsIter)
            );
    }

    // Do all right.
    for (originalIter = detectorAndOriginalImagesRight.begin(),
         canonicalIter = detectorAndWarpedImagesRight.begin(),
         pointsIter = distortedPointsFromCanonicalImagesRight.begin();
         originalIter != detectorAndOriginalImagesRight.end() &&
         canonicalIter != detectorAndWarpedImagesRight.end() &&
         pointsIter != distortedPointsFromCanonicalImagesRight.end();
         ++originalIter,
         ++canonicalIter,
         ++pointsIter
         )
    {
      niftk::ExtractDistortedControlPoints(
            referenceImageData,
            intrinsicRight,
            distortionRight,
            (*originalIter).second,
            (*canonicalIter),
            (*pointsIter)
            );
    }

    // 4. Parameter Fitting: Use the projected control points to refine
    // the camera parameters using Levenberg-Marquardt.
    projectedRMS = niftk::StereoCameraCalibration(
          model,
          pointsFromOriginalImagesLeft,
          pointsFromOriginalImagesRight,
          imageSize,
          intrinsicLeft,
          distortionLeft,
          rvecsLeft,
          tvecsLeft,
          intrinsicRight,
          distortionRight,
          rvecsRight,
          tvecsRight,
          left2RightRotation,
          left2RightTranslation,
          essentialMatrix,
          fundamentalMatrix,
          cv::CALIB_USE_INTRINSIC_GUESS
          );

    std::cout << "Iterative calibration iter=" << count++ << ", prms=" << previousRMS << ", rms=" << projectedRMS << std::endl;

  } while (projectedRMS < previousRMS &&
           fabs(projectedRMS - previousRMS) > 0.005);

  std::cout << "Final stereo calibration, rms=" << projectedRMS << std::endl;
  std::cout << "Final Fxl=" << intrinsicLeft.at<double>(0,0) << std::endl;
  std::cout << "Final Fyl=" << intrinsicLeft.at<double>(1,1) << std::endl;
  std::cout << "Final Cxl=" << intrinsicLeft.at<double>(0,2) << std::endl;
  std::cout << "Final Cyl=" << intrinsicLeft.at<double>(1,2) << std::endl;
  std::cout << "Final K1l=" << distortionLeft.at<double>(0,0) << std::endl;
  std::cout << "Final K2l=" << distortionLeft.at<double>(0,1) << std::endl;
  std::cout << "Final P1l=" << distortionLeft.at<double>(0,2) << std::endl;
  std::cout << "Final P2l=" << distortionLeft.at<double>(0,3) << std::endl;
  std::cout << "Final Fxr=" << intrinsicRight.at<double>(0,0) << std::endl;
  std::cout << "Final Fyr=" << intrinsicRight.at<double>(1,1) << std::endl;
  std::cout << "Final Cxr=" << intrinsicRight.at<double>(0,2) << std::endl;
  std::cout << "Final Cyr=" << intrinsicRight.at<double>(1,2) << std::endl;
  std::cout << "Final K1r=" << intrinsicRight.at<double>(0,0) << std::endl;
  std::cout << "Final K2r=" << intrinsicRight.at<double>(0,1) << std::endl;
  std::cout << "Final P1r=" << intrinsicRight.at<double>(0,2) << std::endl;
  std::cout << "Final P2r=" << intrinsicRight.at<double>(0,3) << std::endl;

  return projectedRMS;
}

} // end namespace
