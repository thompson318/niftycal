/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIterativeCalibrationUtilities_p.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkHomographyUtilities.h"
#include "niftkPointUtilities.h"
#include <highgui.h>
#include <memory>

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

  unsigned int size = list.size();
  std::unique_ptr<ExtractTwoCopiesInfo[]> infos(new ExtractTwoCopiesInfo[size]);

  // Populates a dynamically sized vector as opposed to stl::list, as OpenMP doesn't like stl::list.
  int counter = 0;
  std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::const_iterator iter;
  for (iter = list.begin(); iter != list.end(); ++iter)
  {
    infos[counter].m_DetectorAndImage = &(*iter);
    counter++;
  }

  // Now use OpenMP on the dynamically sized vector.
  #pragma omp parallel shared(info)
  {
    #pragma omp for
    for (int i = 0; i < size; i++)
    {
      infos[i].m_ExtractedPoints = (*(infos[i].m_DetectorAndImage)).first->GetPoints();
      if(infos[i].m_ExtractedPoints.empty())
      {
        niftkNiftyCalThrow() << "All input images should be valid calibration images containing "
                             << "extractable points, and " << counter << " isn't.";
      }
    }
  }

  // Extracts two copies. One is kept constant, and one gets iteratively updated.
  for (int i = 0; i < size; i++)
  {
    a.push_back(infos[i].m_ExtractedPoints);
    b.push_back(infos[i].m_ExtractedPoints);
  }
}


//-----------------------------------------------------------------------------
void ExtractAllDistortedControlPoints(
  const std::pair< cv::Mat, niftk::PointSet>& referenceImageData,
  const cv::Mat& intrinsic,
  const cv::Mat& distortion,
  const std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& originalDetectorsAndImages,
  std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >& canonicalDetectorsAndImages,
  std::list<PointSet>& outputPoints
  )
{
  std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::const_iterator originalIter;
  std::list< std::pair<std::shared_ptr<IPoint2DDetector>, cv::Mat> >::iterator canonicalIter;
  std::list<PointSet>::iterator pointsIter;

  unsigned int size = originalDetectorsAndImages.size();
  std::unique_ptr<ExtractDistortedControlPointsInfo[]> info(new ExtractDistortedControlPointsInfo[size]);

  // Populates a dynamically sized vector as opposed to stl::list, as OpenMP doesn't like stl::list.
  unsigned int counter = 0;
  for (originalIter = originalDetectorsAndImages.begin(),
       canonicalIter = canonicalDetectorsAndImages.begin(),
       pointsIter = outputPoints.begin();
       originalIter != originalDetectorsAndImages.end() &&
       canonicalIter != canonicalDetectorsAndImages.end() &&
       pointsIter != outputPoints.end();
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

  // Now use OpenMP on the dynamically sized vector.
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
}


//-----------------------------------------------------------------------------
void ExtractDistortedControlPoints(
    const std::pair< cv::Mat, niftk::PointSet>& referenceData,
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
  PointSet initialGuess;

  cv::Size2i outputImageSize;
  outputImageSize.width = referenceData.first.cols;
  outputImageSize.height = referenceData.first.rows;

  // 1. Undistort and Unproject: Use the camera parameters to
  // undistort and unproject input images to a canonical pattern.
  cv::undistort(originalImage, undistortedImage, intrinsic, distortion, intrinsic);
  niftk::WarpImageByCorrespondingPoints(
        undistortedImage,
        intrinsic,                     // current estimate (updated each loop)
        distortion,                    // current estimate (updated each loop)
        outputPoints,                  // first time, its a copy of the original detected points,
                                       // after that, its updated, distorted points
        referenceData.second,          // specifies the target point locations
        outputImageSize,               // specifies the proposed size of warped image
        h,                             // output homography, written into
        outputDetectorAndImage.second, // output image, written into
        initialGuess
        );

  // IF the point detector supports it, we can tell it a hint
  // as to indicate what the initial guess should be for whatever
  // point extraction it does in the canonical image space.
  // Probably no use for corner detection, but good for template matching.
  outputDetectorAndImage.first->SetInitialGuess(initialGuess);

  // 2. Localize control points: Localize calibration pattern control
  // points in the canonical pattern.
  cp = outputDetectorAndImage.first->GetPoints();
  if(cp.empty())
  {
    niftkNiftyCalThrow() << "All warped images should still contain valid "
                         << "calibration images containing extractable points.";
  }

  // 3. Reproject: Project the control points using the estimated
  // camera parameters.
  hInv = h.inv(cv::DECOMP_SVD);
  niftk::WarpPointsByHomography(cp, hInv, cpi);
  niftk::DistortPoints(cpi, intrinsic, distortion, cpid);
  niftk::CopyPointsInto(cpid, outputPoints);
}

} // end namespace
