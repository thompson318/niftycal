/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIterativeStereoCameraCalibration.h"
#include "niftkZhangCameraCalibration.h"
#include "niftkTsaiCameraCalibration.h"
#include "niftkStereoCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkHomographyUtilities.h"
#include "niftkPointUtilities.h"
#include <Internal/niftkTriangulationUtilities_p.h>
#include <Internal/niftkIterativeCalibrationUtilities_p.h>
#include <Internal/niftkCalibrationUtilities_p.h>
#include <highgui.h>

namespace niftk
{

//-----------------------------------------------------------------------------
cv::Matx21d IterativeStereoCameraCalibration(
    const Model3D& model,
    const std::pair< cv::Mat, niftk::PointSet>& referenceImageData,
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndOriginalImagesLeft,
    const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndOriginalImagesRight,
    const cv::Size2i& imageSize,
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndCanonicalImagesLeft,
    cv::Mat& intrinsicLeft,
    cv::Mat& distortionLeft,
    std::vector<cv::Mat>& rvecsLeft,
    std::vector<cv::Mat>& tvecsLeft,
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& detectorAndCanonicalImagesRight,
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

  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model is empty.";
  }
  if (detectorAndOriginalImagesLeft.empty())
  {
    niftkNiftyCalThrow() << "Should have at least 1 left-hand view of calibration points.";
  }
  if (detectorAndOriginalImagesRight.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least 1 right-hand view of calibration points.";
  }
  if (detectorAndOriginalImagesLeft.size() != detectorAndOriginalImagesRight.size())
  {
    niftkNiftyCalThrow() << "Should have the same number of views in left and right channel.";
  }

  if (   detectorAndOriginalImagesLeft.size() != detectorAndOriginalImagesRight.size()
      || detectorAndOriginalImagesLeft.size() != detectorAndCanonicalImagesLeft.size()
      || detectorAndOriginalImagesLeft.size() != detectorAndCanonicalImagesRight.size()
      || detectorAndOriginalImagesRight.size() != detectorAndCanonicalImagesLeft.size()
      || detectorAndOriginalImagesRight.size() != detectorAndCanonicalImagesRight.size()
      || detectorAndCanonicalImagesLeft.size() != detectorAndCanonicalImagesRight.size()
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

  // 1. Detect control points: Detect calibration pattern control
  // points (corners, circle or ring centers) in the input images.

  std::list<PointSet> pointsFromOriginalImagesLeft;
  std::list<PointSet> distortedPointsFromCanonicalImagesLeft;
  std::list<PointSet> bestPointSetsSoFarLeft;

  ExtractTwoCopiesOfControlPoints(detectorAndOriginalImagesLeft,
                                  pointsFromOriginalImagesLeft,
                                  distortedPointsFromCanonicalImagesLeft
                                  );

  std::list<PointSet> pointsFromOriginalImagesRight;
  std::list<PointSet> distortedPointsFromCanonicalImagesRight;
  std::list<PointSet> bestPointSetsSoFarRight;

  ExtractTwoCopiesOfControlPoints(detectorAndOriginalImagesRight,
                                  pointsFromOriginalImagesRight,
                                  distortedPointsFromCanonicalImagesRight
                                  );

  // 2. Parameter Fitting: Use the detected control points to estimate
  // camera parameters using Levenberg-Marquardt.

  #pragma omp sections
  {
    #pragma omp section
    {
      if (pointsFromOriginalImagesLeft.size() == 1)
      {
        cv::Point2d sensorDimensions(1,1);

        niftk::TsaiMonoCameraCalibration(
          model,
          *(pointsFromOriginalImagesLeft.begin()),
          imageSize,
          sensorDimensions,
          intrinsicLeft,
          distortionLeft,
          rvecsLeft[0],
          tvecsLeft[0],
          true
          );
      }
      else
      {
        niftk::ZhangMonoCameraCalibration(
          model,
          pointsFromOriginalImagesLeft,
          imageSize,
          intrinsicLeft,
          distortionLeft,
          rvecsLeft,
          tvecsLeft,
          cvFlags
          );
      }
    }
     
    #pragma omp section
    {
      if (pointsFromOriginalImagesRight.size() == 1)
      {
        cv::Point2d sensorDimensions(1,1);

        niftk::TsaiMonoCameraCalibration(
          model,
          *(pointsFromOriginalImagesRight.begin()),
          imageSize,
          sensorDimensions,
          intrinsicRight,
          distortionRight,
          rvecsRight[0],
          tvecsRight[0],
          true
          );
      }
      else
      {
        niftk::ZhangMonoCameraCalibration(
          model,
          pointsFromOriginalImagesRight,
          imageSize,
          intrinsicRight,
          distortionRight,
          rvecsRight,
          tvecsRight,
          cvFlags
          );
      }
    }
  }

  int iterativeCvFlags = cvFlags | cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_INTRINSIC;

  if (pointsFromOriginalImagesLeft.size() == 1)
  {
    result = niftk::TsaiStereoCameraCalibration(
      model,
      *(pointsFromOriginalImagesLeft.begin()),
      *(pointsFromOriginalImagesRight.begin()),
      imageSize,
      intrinsicLeft,
      distortionLeft,
      rvecsLeft[0],
      tvecsLeft[0],
      intrinsicRight,
      distortionRight,
      rvecsRight[0],
      tvecsRight[0],
      leftToRightRotationMatrix,
      leftToRightTranslationVector,
      essentialMatrix,
      fundamentalMatrix,
      iterativeCvFlags,
      optimise3D
      );
  }
  else
  {
    result = niftk::StereoCameraCalibration(
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
      leftToRightRotationMatrix,
      leftToRightTranslationVector,
      essentialMatrix,
      fundamentalMatrix,
      iterativeCvFlags,
      optimise3D
      );
  }

  bestPointSetsSoFarLeft = pointsFromOriginalImagesLeft;
  bestPointSetsSoFarRight = pointsFromOriginalImagesRight;

  std::cout << "Initial stereo calibration, rms2D=" << result(0,0)
            << ", rms3D=" << result(1,0)
            << std::endl;

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
  cv::Mat rvec;
  cv::Rodrigues(leftToRightRotationMatrix, rvec);
  std::cout << "Initial R1=" << rvec.at<double>(0,0) << std::endl;
  std::cout << "Initial R2=" << rvec.at<double>(0,1) << std::endl;
  std::cout << "Initial R3=" << rvec.at<double>(0,2) << std::endl;
  std::cout << "Initial T1=" << leftToRightTranslationVector.at<double>(0,0) << std::endl;
  std::cout << "Initial T2=" << leftToRightTranslationVector.at<double>(0,1) << std::endl;
  std::cout << "Initial T3=" << leftToRightTranslationVector.at<double>(0,2) << std::endl;
  std::cout << std::endl;

  unsigned int iterationCount = 0;
  double currentRMS = 0;
  cv::Matx21d currentResult;

  if (optimise3D)
  {
    currentRMS = result(1, 0);
  }
  else
  {
    currentRMS = result(0, 0);
  }
  double previousRMS = std::numeric_limits<double>::max();

  while (currentRMS < previousRMS && fabs(currentRMS - previousRMS) > 0.0005)
  {
    previousRMS = currentRMS;

    niftk::ExtractAllDistortedControlPoints(
          referenceImageData,
          intrinsicLeft,
          distortionLeft,
          imageSize,
          detectorAndOriginalImagesLeft,
          detectorAndCanonicalImagesLeft,
          distortedPointsFromCanonicalImagesLeft
          );

    niftk::ExtractAllDistortedControlPoints(
          referenceImageData,
          intrinsicRight,
          distortionRight,
          imageSize,
          detectorAndOriginalImagesRight,
          detectorAndCanonicalImagesRight,
          distortedPointsFromCanonicalImagesRight
          );

    // 4. Parameter Fitting: Use the projected control points to refine
    // the camera parameters using Levenberg-Marquardt.
    cv::Mat tmpIntrinsicLeft = intrinsicLeft.clone();
    cv::Mat tmpDistortionLeft = distortionLeft.clone();
    cv::Mat tmpIntrinsicRight = intrinsicRight.clone();
    cv::Mat tmpDistortionRight = distortionRight.clone();
    cv::Mat tmpLeftToRightRotationMatrix = leftToRightRotationMatrix.clone();
    cv::Mat tmpLeftToRightTranslationVector = leftToRightTranslationVector.clone();
    cv::Mat tmpEssentialMatrix = essentialMatrix.clone();
    cv::Mat tmpFundamentalMatrix = fundamentalMatrix.clone();

    if (distortedPointsFromCanonicalImagesLeft.size() == 1)
    {
      currentResult = niftk::TsaiStereoCameraCalibration(
        model,
        *(distortedPointsFromCanonicalImagesLeft.begin()),
        *(distortedPointsFromCanonicalImagesRight.begin()),
        imageSize,
        tmpIntrinsicLeft,
        tmpDistortionLeft,
        rvecsLeft[0],
        tvecsLeft[0],
        tmpIntrinsicRight,
        tmpDistortionRight,
        rvecsRight[0],
        tvecsRight[0],
        tmpLeftToRightRotationMatrix,
        tmpLeftToRightTranslationVector,
        tmpEssentialMatrix,
        tmpFundamentalMatrix,
        iterativeCvFlags,
        optimise3D
       );
    }
    else
    {
      currentResult = niftk::StereoCameraCalibration(
        model,
        distortedPointsFromCanonicalImagesLeft,
        distortedPointsFromCanonicalImagesRight,
        imageSize,
        tmpIntrinsicLeft,
        tmpDistortionLeft,
        rvecsLeft,
        tvecsLeft,
        tmpIntrinsicRight,
        tmpDistortionRight,
        rvecsRight,
        tvecsRight,
        tmpLeftToRightRotationMatrix,
        tmpLeftToRightTranslationVector,
        tmpEssentialMatrix,
        tmpFundamentalMatrix,
        iterativeCvFlags,
        optimise3D
       );
    }

    if (optimise3D)
    {
      currentRMS = currentResult(1, 0);
    }
    else
    {
      currentRMS = currentResult(0, 0);
    }

    std::cout << "Iterative calibration iter=" << iterationCount++
              << ", prms=" << previousRMS
              << ", crms=" << currentRMS
              << std::endl;

    if (currentRMS < previousRMS)
    {
      result = currentResult;

      tmpIntrinsicLeft.copyTo(intrinsicLeft);
      tmpDistortionLeft.copyTo(distortionLeft);
      tmpIntrinsicRight.copyTo(intrinsicRight);
      tmpDistortionRight.copyTo(distortionRight);
      tmpLeftToRightRotationMatrix.copyTo(leftToRightRotationMatrix);
      tmpLeftToRightTranslationVector.copyTo(leftToRightTranslationVector);
      tmpEssentialMatrix.copyTo(essentialMatrix);
      tmpFundamentalMatrix.copyTo(fundamentalMatrix);

      bestPointSetsSoFarLeft = distortedPointsFromCanonicalImagesLeft;
      bestPointSetsSoFarRight = distortedPointsFromCanonicalImagesRight;
    }
    else
    {
      currentRMS = previousRMS;
    }
  } // end while

  niftk::ComputeStereoExtrinsics(rvecsLeft,
                                 tvecsLeft,
                                 leftToRightRotationMatrix,
                                 leftToRightTranslationVector,
                                 rvecsRight,
                                 tvecsRight
                                );

  std::cout << "Iterative calibration finished, rms=" << currentRMS
            << ", over " << bestPointSetsSoFarLeft.size()
            << ", left PointSets and " << bestPointSetsSoFarRight.size()
            << ", right PointSets."
            << std::endl;

  std::cout << "Final stereo calibration, rms2D=" << result(0,0)
            << ", rms3D=" << result(1,0)
            << std::endl;
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
  cv::Rodrigues(leftToRightRotationMatrix, rvec);
  std::cout << "Final R1=" << rvec.at<double>(0,0) << std::endl;
  std::cout << "Final R2=" << rvec.at<double>(0,1) << std::endl;
  std::cout << "Final R3=" << rvec.at<double>(0,2) << std::endl;
  std::cout << "Final T1=" << leftToRightTranslationVector.at<double>(0,0) << std::endl;
  std::cout << "Final T2=" << leftToRightTranslationVector.at<double>(0,1) << std::endl;
  std::cout << "Final T3=" << leftToRightTranslationVector.at<double>(0,2) << std::endl;

  return result;
}

} // end namespace
