/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRingsPointDetector.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkHomographyUtilities.h"
#include "niftkMatrixUtilities.h"
#include "niftkTemplateMatching.h"
#include <cv.h>
#include <highgui.h>

namespace niftk {

//-----------------------------------------------------------------------------
RingsPointDetector::RingsPointDetector(cv::Size2i patternSize,
                                       cv::Size2i offsetForTemplate
                                      )
: TemplateMatchingPointDetector(patternSize, offsetForTemplate)
{
}


//-----------------------------------------------------------------------------
RingsPointDetector::~RingsPointDetector()
{
}


//-----------------------------------------------------------------------------
void RingsPointDetector::ExtractBlobs(const cv::Mat& image,
                                            cv::Mat& bigBlobs,
                                            cv::Mat& littleBlobs
                                           )
{

  cv::Mat thresholdedImage;
  cv::threshold(image, thresholdedImage, 50, 255, cv::THRESH_BINARY);

  // cv::RETR_CCOMP is important.
  //
  // "CV_RETR_CCOMP retrieves all of the contours and organizes
  // them into a two-level hierarchy. At the top level, there are
  // external boundaries of the components. At the second level,
  // there are boundaries of the holes. If there is another contour
  // inside a hole of a connected component, it is still put at the top level."
  //
  // And using RETR_CCOMP, the inner holes come out first.

  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<unsigned int> innerRingIndexes;
  std::vector<unsigned int> outerRingIndexes;

  cv::findContours(thresholdedImage, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

  unsigned int counter = 0;
  while(   counter < hierarchy.size()
        && hierarchy[counter][2] == -1 // no parent, so top level object.
        && hierarchy[counter][0] != -1 // has a next item, so should stop, when we hit the external contour.
        )
  {
    innerRingIndexes.push_back(counter++);
  }
  counter++;
  while(   counter < hierarchy.size()
        && hierarchy[counter][3] != -1) // is child, so its an outer ring, which is a child of the outmost contour.
  {
    outerRingIndexes.push_back(counter++);
  }

  if (   innerRingIndexes.size() != (m_PatternSize.width * m_PatternSize.height)
      || outerRingIndexes.size() != (m_PatternSize.width * m_PatternSize.height))
  {
    return;
  }

  cv::Scalar black( 0, 0, 0);

  bigBlobs = image.clone();
  bigBlobs.setTo(255);
  for (int i = 0; i < outerRingIndexes.size(); i++)
  {
    drawContours( bigBlobs, contours, outerRingIndexes[i], black, CV_FILLED, 8, hierarchy );
  }

  littleBlobs = image.clone();
  littleBlobs.setTo(255);
  for (int i = 0; i < innerRingIndexes.size(); i++)
  {
    drawContours( littleBlobs, contours, innerRingIndexes[i], black, CV_FILLED, 8, hierarchy );
  }
}


//-----------------------------------------------------------------------------
PointSet RingsPointDetector::GetPointsUsingContours(const cv::Mat& image)
{
  PointSet result;
  unsigned int numberOfRings= m_PatternSize.width * m_PatternSize.height;

  cv::Mat bigBlobs;    // outer circle, but filled in
  cv::Mat littleBlobs; // inner circle
  this->ExtractBlobs(image, bigBlobs, littleBlobs);

  cv::SimpleBlobDetector::Params params;
  params.maxArea = m_MaxAreaInPixels;
  cv::Ptr<cv::FeatureDetector> blobDetector = new cv::SimpleBlobDetector(params);

  std::vector<cv::Point2f> bigCentres;
  bool foundBig = cv::findCirclesGrid(
    bigBlobs, m_PatternSize, bigCentres,
    cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING
    , blobDetector
    );

  std::vector<cv::Point2f> littleCentres;
  bool foundLittle = cv::findCirclesGrid(
    littleBlobs, m_PatternSize, littleCentres,
    cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING
    , blobDetector
    );

  if (!foundBig || !foundLittle)
  {
    return result;
  }

  assert(littleCentres.size() == bigCentres.size());

  if (   littleCentres.size() == numberOfRings
      && bigCentres.size() == numberOfRings
      )
  {
    for ( unsigned int k = 0; k < numberOfRings; ++k)
    {
      Point2D tmp;
      tmp.point.x = (littleCentres[k].x + bigCentres[k].x)/2.0;
      tmp.point.y = (littleCentres[k].y + bigCentres[k].y)/2.0;
      tmp.id = k;
      result.insert(IdPoint2D(tmp.id, tmp));
    }
  }
  return result;
}

} // end namespace
