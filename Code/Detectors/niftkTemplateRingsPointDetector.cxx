/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkTemplateRingsPointDetector.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkHomographyUtilities.h>
#include <niftkMatrixUtilities.h>
#include <niftkTemplateMatching.h>
#include <cv.h>
#include <highgui.h>

namespace niftk {

//-----------------------------------------------------------------------------
TemplateRingsPointDetector::TemplateRingsPointDetector(cv::Size2i patternSize,
                                                       cv::Size2i offsetForTemplate,
                                                       int flags
                                                      )
: TemplateMatchingPointDetector(patternSize, offsetForTemplate, flags)
, m_UseOuterContour(true)
, m_ThresholdValue(50)
, m_AdaptiveThreshold(20)
{
}


//-----------------------------------------------------------------------------
TemplateRingsPointDetector::~TemplateRingsPointDetector()
{
}


//-----------------------------------------------------------------------------
void TemplateRingsPointDetector::SetUseOuterContour(const bool& useIt)
{
  m_UseOuterContour = useIt;
}


//-----------------------------------------------------------------------------
void TemplateRingsPointDetector::ExtractIndexes(const cv::Mat& image,
                                                std::vector<cv::Vec4i>& hierarchy,
                                                std::vector<std::vector<cv::Point> >& contours,
                                                std::vector<unsigned int>& innerRingIndexes,
                                                std::vector<unsigned int>& outerRingIndexes
                                               )
{
  innerRingIndexes.clear();
  outerRingIndexes.clear();

  // cv::RETR_CCOMP is important.
  //
  // "CV_RETR_CCOMP retrieves all of the contours and organizes
  // them into a two-level hierarchy. At the top level, there are
  // external boundaries of the components. At the second level,
  // there are boundaries of the holes. If there is another contour
  // inside a hole of a connected component, it is still put at the top level."
  //
  // And using RETR_CCOMP, the inner holes come out first.

  cv::findContours(image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

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
}


//-----------------------------------------------------------------------------
void TemplateRingsPointDetector::ExtractBlobs(const cv::Mat& image,
                                              cv::Mat& bigBlobs,
                                              cv::Mat& littleBlobs
                                             )
{

  cv::Mat thresholdedImage;
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point> > contours;
  std::vector<unsigned int> innerRingIndexes;
  std::vector<unsigned int> outerRingIndexes;

  bigBlobs = image.clone();
  bigBlobs.setTo(255);
  littleBlobs = image.clone();
  littleBlobs.setTo(255);
  cv::Scalar black( 0, 0, 0);

  cv::threshold(image, thresholdedImage, m_ThresholdValue, 255, cv::THRESH_BINARY);

  this->ExtractIndexes(thresholdedImage, hierarchy, contours, innerRingIndexes, outerRingIndexes);

  if (   innerRingIndexes.size() != (m_PatternSize.width * m_PatternSize.height)
      || outerRingIndexes.size() != (m_PatternSize.width * m_PatternSize.height))
  {
    int blockSize = image.cols / m_PatternSize.width;
    if (blockSize %2 == 0)
    {
      blockSize += 1;
    }
    cv::adaptiveThreshold(image, thresholdedImage, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY, blockSize, m_AdaptiveThreshold);

    this->ExtractIndexes(thresholdedImage, hierarchy, contours, innerRingIndexes, outerRingIndexes);

    // Issue #44: We might as well keep going at this point,
    //            draw the bigBlobs and littleBlobs and see
    //            what the blob detector makes of it.
    // if (   innerRingIndexes.size() != (m_PatternSize.width * m_PatternSize.height)
    //     || outerRingIndexes.size() != (m_PatternSize.width * m_PatternSize.height))
    // {
    //   return;
    // }
  }

  for (int i = 0; i < outerRingIndexes.size(); i++)
  {
    drawContours( bigBlobs, contours, outerRingIndexes[i], black, CV_FILLED, 8, hierarchy );
  }
  for (int i = 0; i < innerRingIndexes.size(); i++)
  {
    drawContours( littleBlobs, contours, innerRingIndexes[i], black, CV_FILLED, 8, hierarchy );
  }
}


//-----------------------------------------------------------------------------
PointSet TemplateRingsPointDetector::GetPointsUsingContours(const cv::Mat& image)
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
    this->GetFlags(), blobDetector
    );

  std::vector<cv::Point2f> littleCentres;
  bool foundLittle = cv::findCirclesGrid(
    littleBlobs, m_PatternSize, littleCentres,
    this->GetFlags(), blobDetector
    );

  if (!foundBig && !foundLittle)
  {
    return result;
  }

  for ( unsigned int k = 0; k < numberOfRings; ++k)
  {
    Point2D tmp;
    tmp.point.x = 0;
    tmp.point.y = 0;

    unsigned int counter = 0;
    if (m_UseOuterContour && foundBig && k < bigCentres.size())
    {
      tmp.point.x += bigCentres[k].x;
      tmp.point.y += bigCentres[k].y;
      counter++;
    }
    if (foundLittle && k < littleCentres.size())
    {
      tmp.point.x += littleCentres[k].x;
      tmp.point.y += littleCentres[k].y;
      counter++;
    }
    assert(counter > 0); // because one, or other, or both lists should have point.
    tmp.point.x /= static_cast<double>(counter);
    tmp.point.y /= static_cast<double>(counter);
    tmp.id = k;
    result.insert(IdPoint2D(tmp.id, tmp));
  }
  return result;
}

} // end namespace
