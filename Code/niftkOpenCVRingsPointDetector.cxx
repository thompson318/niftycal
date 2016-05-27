/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkOpenCVRingsPointDetector.h"
#include "niftkNiftyCalExceptionMacro.h"
#include <cv.h>
#include <highgui.h>

namespace niftk {

//-----------------------------------------------------------------------------
OpenCVRingsPointDetector::OpenCVRingsPointDetector(cv::Size2i patternSize)
: m_PatternSize(patternSize)
, m_UseContours(true)
, m_UseTemplateMatching(false)
{
  if (m_PatternSize.width < 2)
  {
    niftkNiftyCalThrow() << "Number of circles in width axes is < 2.";
  }
  if (m_PatternSize.height < 2)
  {
    niftkNiftyCalThrow() << "Number of circles in height axes is < 2.";
  }
}


//-----------------------------------------------------------------------------
OpenCVRingsPointDetector::~OpenCVRingsPointDetector()
{
}


//-----------------------------------------------------------------------------
void OpenCVRingsPointDetector::SetUseContours(bool useContours)
{
  m_UseContours = useContours;
}


//-----------------------------------------------------------------------------
void OpenCVRingsPointDetector::SetUseTemplateMatching(bool useTemplateMatching)
{
  m_UseTemplateMatching = useTemplateMatching;
}


//-----------------------------------------------------------------------------
void OpenCVRingsPointDetector::ExtractBlobs(const cv::Mat& image,
                                            cv::Mat& bigBlobs,
                                            cv::Mat& littleBlobs
                                           )
{

  cv::Mat thresholdedImage;
  cv::threshold(image, thresholdedImage, 127.5, 255, cv::THRESH_BINARY);

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
PointSet OpenCVRingsPointDetector::GetPointsUsingContours(const cv::Mat& image)
{
  PointSet result;
  unsigned int numberOfRings= m_PatternSize.width * m_PatternSize.height;

  cv::Mat bigBlobs;    // outer circle, but filled in
  cv::Mat littleBlobs; // inner circle
  this->ExtractBlobs(image, bigBlobs, littleBlobs);

  cv::SimpleBlobDetector::Params params;
  params.maxArea = 10e4;
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

  // Output
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


//-----------------------------------------------------------------------------
PointSet OpenCVRingsPointDetector::GetPointsUsingTemplateMatching(
    const cv::Mat& image, const niftk::PointSet& startingGuess)
{
  return m_CachedPoints;
}


//-----------------------------------------------------------------------------
PointSet OpenCVRingsPointDetector::InternalGetPoints(const cv::Mat& imageToUse)
{
  niftk::PointSet result;
  if (!m_UseContours && !m_UseTemplateMatching)
  {
    return result;
  }

  if (m_CachedPoints.size() == 0 || m_UseContours)
  {
    m_CachedPoints = this->GetPointsUsingContours(imageToUse);
  }

  if (m_UseTemplateMatching)
  {
    m_CachedPoints = this->GetPointsUsingTemplateMatching(imageToUse, m_CachedPoints);
  }
  result = m_CachedPoints;
  return result;
}

} // end namespace
