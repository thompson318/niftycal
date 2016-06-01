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
#include "niftkIterativeCalibrationUtilities_p.h"
#include "niftkHomographyUtilities.h"
#include "niftkPointUtilities.h"

#include <highgui.h>
#include <memory>

namespace niftk
{

//-----------------------------------------------------------------------------
double IterativeMonoCameraCalibration(
    const Model3D& model,
    const std::pair< cv::Mat, niftk::PointSet>& referenceImageData,
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
    niftkNiftyCalThrow() << "Model is empty.";
  }
  if (detectorAndOriginalImages.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least 2 views of calibration points.";
  }
  if (detectorAndOriginalImages.size() != detectorAndWarpedImages.size())
  {
    niftkNiftyCalThrow() << "detectorAndOriginalImages must be the same size as detectorAndWarpedImages.";
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
  std::list<PointSet> pointsFromOriginalImages;
  std::list<PointSet> distortedPointsFromCanonicalImages;

  ExtractTwoCopiesOfControlPoints(detectorAndOriginalImages,
                                  pointsFromOriginalImages,
                                  distortedPointsFromCanonicalImages
                                  );

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

  unsigned int count = 0;
  int iterativeCvFlags = cvFlags | cv::CALIB_USE_INTRINSIC_GUESS;
  double previousRMS = std::numeric_limits<double>::max();

  while (projectedRMS < previousRMS && fabs(projectedRMS - previousRMS) > 0.0005)
  {
    previousRMS = projectedRMS;

    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::const_iterator originalIter;
    std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::iterator canonicalIter;
    std::list<PointSet>::iterator pointsIter;

    unsigned int size = detectorAndOriginalImages.size();

    std::unique_ptr<ExtractDistortedControlPointsInfo[]> info(new ExtractDistortedControlPointsInfo[size]);
    unsigned int counter = 0;
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
      info[counter].m_OriginalImage = &((*originalIter).second);
      info[counter].m_DetectorAndImage = &(*canonicalIter);
      info[counter].m_OutputPoints = &(*pointsIter);
      counter++;
    }

    #pragma omp parallel shared(referenceImageData), shared(intrinsic), shared(distortion), shared(info)
    {
      #pragma omp for
      for (counter = 0; counter < size; counter++)
      {
        niftk::ExtractDistortedControlPoints(
          referenceImageData,
          intrinsic,
          distortion,
          *(info[counter].m_OriginalImage),
          *(info[counter].m_DetectorAndImage),
          *(info[counter].m_OutputPoints)
        );
      }
    }

    // 4. Parameter Fitting: Use the projected control points to refine
    // the camera parameters using Levenberg-Marquardt.
    cv::Mat tmpIntrinsic = intrinsic.clone();
    cv::Mat tmpDistortion = distortion.clone();
    projectedRMS = niftk::MonoCameraCalibration(
          model,
          distortedPointsFromCanonicalImages,
          imageSize,
          tmpIntrinsic,
          tmpDistortion,
          rvecs,
          tvecs,
          iterativeCvFlags
          );

    std::cout << "Iterative calibration iter=" << count++
              << ", prms=" << previousRMS
              << ", rms=" << projectedRMS
              << std::endl;

    if (projectedRMS < previousRMS)
    {
      tmpIntrinsic.copyTo(intrinsic);
      tmpDistortion.copyTo(distortion);
    }
    else
    {
      projectedRMS = previousRMS;
    }
  } // end while

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
