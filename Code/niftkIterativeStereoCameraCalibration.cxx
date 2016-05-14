/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIterativeStereoCameraCalibration.h"
#include "niftkMonoCameraCalibration.h"
#include "niftkStereoCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkIterativeCalibrationUtilities_p.h"
#include "niftkHomographyUtilities.h"
#include "niftkPointUtilities.h"
#include <highgui.h>

namespace niftk
{

//-----------------------------------------------------------------------------
double IterativeStereoCameraCalibration(
    const Model3D& model,
    const std::pair< cv::Mat, niftk::PointSet>& referenceImageData,
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
    cv::Mat& rightToLeftRotation,
    cv::Mat& rightToLeftTranslation,
    cv::Mat& essentialMatrix,
    cv::Mat& fundamentalMatrix,
    const int& cvFlags
    )
{
  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model is empty.";
  }
  if (detectorAndOriginalImagesLeft.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least 2 left-hand views of calibration points.";
  }
  if (detectorAndOriginalImagesRight.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least 2 right-hand views of calibration points.";
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
  if (referenceImageData.first.cols == 0 || referenceImageData.first.rows == 0)
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

  ExtractTwoCopiesOfControlPoints(detectorAndOriginalImagesLeft,
                                  pointsFromOriginalImagesLeft,
                                  distortedPointsFromCanonicalImagesLeft
                                  );

  std::list<PointSet> pointsFromOriginalImagesRight;
  std::list<PointSet> distortedPointsFromCanonicalImagesRight;

  ExtractTwoCopiesOfControlPoints(detectorAndOriginalImagesRight,
                                  pointsFromOriginalImagesRight,
                                  distortedPointsFromCanonicalImagesRight
                                  );

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

  int iterativeCvFlags = cvFlags | cv::CALIB_USE_INTRINSIC_GUESS;

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
        rightToLeftRotation,
        rightToLeftTranslation,
        essentialMatrix,
        fundamentalMatrix,
        iterativeCvFlags
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
  std::cout << std::endl;
  std::cout << "Initial Fxr=" << intrinsicRight.at<double>(0,0) << std::endl;
  std::cout << "Initial Fyr=" << intrinsicRight.at<double>(1,1) << std::endl;
  std::cout << "Initial Cxr=" << intrinsicRight.at<double>(0,2) << std::endl;
  std::cout << "Initial Cyr=" << intrinsicRight.at<double>(1,2) << std::endl;
  std::cout << "Initial K1r=" << distortionRight.at<double>(0,0) << std::endl;
  std::cout << "Initial K2r=" << distortionRight.at<double>(0,1) << std::endl;
  std::cout << "Initial P1r=" << distortionRight.at<double>(0,2) << std::endl;
  std::cout << "Initial P2r=" << distortionRight.at<double>(0,3) << std::endl;
  std::cout << std::endl;
  std::cout << "Initial R1=" << rightToLeftRotation.at<double>(0,0) << std::endl;
  std::cout << "Initial R2=" << rightToLeftRotation.at<double>(0,1) << std::endl;
  std::cout << "Initial R3=" << rightToLeftRotation.at<double>(0,2) << std::endl;
  std::cout << "Initial T1=" << rightToLeftTranslation.at<double>(0,0) << std::endl;
  std::cout << "Initial T2=" << rightToLeftTranslation.at<double>(0,1) << std::endl;
  std::cout << "Initial T3=" << rightToLeftTranslation.at<double>(0,2) << std::endl;
  std::cout << std::endl;

  //return projectedRMS;

  unsigned int count = 0;
  double previousRMS = std::numeric_limits<double>::max();

  while (projectedRMS < previousRMS && fabs(projectedRMS - previousRMS) > 0.0005)
  {
    previousRMS = projectedRMS;

    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::const_iterator originalIter;
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::iterator canonicalIter;
    std::list<PointSet>::iterator pointsIter;

    std::list<PointSet> trimmedPointsLeft;
    std::list<PointSet> trimmedPointsRight;

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
      trimmedPointsLeft.push_back(
            niftk::ExtractDistortedControlPoints(
            referenceImageData,
            intrinsicLeft,
            distortionLeft,
            (*originalIter).second,
            (*canonicalIter),
            (*pointsIter)
            )
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
      trimmedPointsRight.push_back(
            niftk::ExtractDistortedControlPoints(
            referenceImageData,
            intrinsicRight,
            distortionRight,
            (*originalIter).second,
            (*canonicalIter),
            (*pointsIter)
            )
      );
    }

    // 4. Parameter Fitting: Use the projected control points to refine
    // the camera parameters using Levenberg-Marquardt.
    cv::Mat tmpIntrinsicLeft = intrinsicLeft.clone();
    cv::Mat tmpDistortionLeft = distortionLeft.clone();
    cv::Mat tmpIntrinsicRight = intrinsicRight.clone();
    cv::Mat tmpDistortionRight = distortionRight.clone();
    cv::Mat tmpRightToLeftRotation = rightToLeftRotation.clone();
    cv::Mat tmpRightToLeftTranslation = rightToLeftTranslation.clone();
    cv::Mat tmpEssentialMatrix = essentialMatrix.clone();
    cv::Mat tmpFundamentalMatrix = fundamentalMatrix.clone();

    projectedRMS = niftk::StereoCameraCalibration(
          model,
          distortedPointsFromCanonicalImagesLeft,  // or used trimmedPointsLeft?
          distortedPointsFromCanonicalImagesRight, // or use trimmedPointsRight?
          imageSize,
          tmpIntrinsicLeft,
          tmpDistortionLeft,
          rvecsLeft,
          tvecsLeft,
          tmpIntrinsicRight,
          tmpDistortionRight,
          rvecsRight,
          tvecsRight,
          tmpRightToLeftRotation,
          tmpRightToLeftTranslation,
          tmpEssentialMatrix,
          tmpFundamentalMatrix,
          iterativeCvFlags
          );

    std::cout << "Iterative calibration iter=" << count++
              << ", prms=" << previousRMS
              << ", rms=" << projectedRMS
              << std::endl;

    if (projectedRMS < previousRMS)
    {
      tmpIntrinsicLeft.copyTo(intrinsicLeft);
      tmpDistortionLeft.copyTo(distortionLeft);
      tmpIntrinsicRight.copyTo(intrinsicRight);
      tmpDistortionRight.copyTo(distortionRight);
      tmpRightToLeftRotation.copyTo(rightToLeftRotation);
      tmpRightToLeftTranslation.copyTo(rightToLeftTranslation);
      tmpEssentialMatrix.copyTo(essentialMatrix);
      tmpFundamentalMatrix.copyTo(fundamentalMatrix);
    }
    else
    {
      projectedRMS = previousRMS;
    }
  } // end while

  std::cout << "Final stereo calibration, rms=" << projectedRMS << std::endl;
  std::cout << "Final Fxl=" << intrinsicLeft.at<double>(0,0) << std::endl;
  std::cout << "Final Fyl=" << intrinsicLeft.at<double>(1,1) << std::endl;
  std::cout << "Final Cxl=" << intrinsicLeft.at<double>(0,2) << std::endl;
  std::cout << "Final Cyl=" << intrinsicLeft.at<double>(1,2) << std::endl;
  std::cout << "Final K1l=" << distortionLeft.at<double>(0,0) << std::endl;
  std::cout << "Final K2l=" << distortionLeft.at<double>(0,1) << std::endl;
  std::cout << "Final P1l=" << distortionLeft.at<double>(0,2) << std::endl;
  std::cout << "Final P2l=" << distortionLeft.at<double>(0,3) << std::endl;
  std::cout << std::endl;
  std::cout << "Final Fxr=" << intrinsicRight.at<double>(0,0) << std::endl;
  std::cout << "Final Fyr=" << intrinsicRight.at<double>(1,1) << std::endl;
  std::cout << "Final Cxr=" << intrinsicRight.at<double>(0,2) << std::endl;
  std::cout << "Final Cyr=" << intrinsicRight.at<double>(1,2) << std::endl;
  std::cout << "Final K1r=" << distortionRight.at<double>(0,0) << std::endl;
  std::cout << "Final K2r=" << distortionRight.at<double>(0,1) << std::endl;
  std::cout << "Final P1r=" << distortionRight.at<double>(0,2) << std::endl;
  std::cout << "Final P2r=" << distortionRight.at<double>(0,3) << std::endl;
  std::cout << std::endl;
  std::cout << "Final R1=" << rightToLeftRotation.at<double>(0,0) << std::endl;
  std::cout << "Final R2=" << rightToLeftRotation.at<double>(0,1) << std::endl;
  std::cout << "Final R3=" << rightToLeftRotation.at<double>(0,2) << std::endl;
  std::cout << "Final T1=" << rightToLeftTranslation.at<double>(0,0) << std::endl;
  std::cout << "Final T2=" << rightToLeftTranslation.at<double>(0,1) << std::endl;
  std::cout << "Final T3=" << rightToLeftTranslation.at<double>(0,2) << std::endl;

  return projectedRMS;
}

} // end namespace
