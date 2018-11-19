/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkTemplateMatching.h"
#include "niftkMatrixUtilities.h"

namespace niftk {

//-----------------------------------------------------------------------------
PointSet DoTemplateMatchingForAllPoints(const cv::Mat& image,
                                        const cv::Mat& templateImage,
                                        const cv::Size2i& offset,
                                        const niftk::PointSet& startingPoints
                                       )
{
  // This is the output.
  niftk::PointSet updatedPoints;

  cv::Size2i templateSize = templateImage.size();

  cv::Point2d halfTemplateSize;
  halfTemplateSize.x = (templateSize.width - 1)/2.0;
  halfTemplateSize.y = (templateSize.height - 1)/2.0;

  // Copy template into float image - once.
  cv::Mat templateAsFloat = cv::Mat::zeros(templateImage.cols, templateImage.rows, CV_8U);
  templateImage.convertTo(templateAsFloat, CV_8U);

  // This is to store a copy of the image data, as we need float.
  cv::Mat regionAsFloat = cv::Mat::zeros(2 * offset.width + templateSize.width,
                                      2 * offset.height + templateSize.height,
                                      CV_8U
                                     );

  // This is the results array, temporary storage, for output of template matching.
  cv::Mat result = cv::Mat::zeros(regionAsFloat.cols - templateAsFloat.cols + 1,
                               regionAsFloat.rows - templateAsFloat.rows + 1,
                               CV_8U
                              );

  cv::Point2d startingOffset;
  startingOffset.x = offset.width + halfTemplateSize.x;
  startingOffset.y = offset.height + halfTemplateSize.y;

  double minVal = 0;
  double maxVal = 0;
  cv::Point minLoc;
  cv::Point maxLoc;
  cv::Point matchLoc;
  cv::Point2d originalPoint;
  cv::Point2d interpolatedPoint;
  niftk::Point2D p;
  cv::Matx33d surface;

  niftk::PointSet::const_iterator iter;
  for (iter  = startingPoints.begin();
       iter != startingPoints.end();
       ++iter
       )
  {
    originalPoint = (*iter).second.point;
    niftk::NiftyCalIdType id = (*iter).first;

    cv::Rect rect(static_cast<int>(originalPoint.x - startingOffset.x),
                  static_cast<int>(originalPoint.y - startingOffset.y),
                  regionAsFloat.cols,
                  regionAsFloat.rows
                  );

    cv::Mat region(image, rect);
    region.convertTo(regionAsFloat, CV_8U);

    cv::matchTemplate( regionAsFloat, templateAsFloat, result, cv::TM_CCOEFF_NORMED );
    cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );
    matchLoc = maxLoc;

    for (int r = 0; r < 3; r++)
    {
      for (int c = 0; c < 3; c++)
      {
        surface(r, c) = result.at<float>(matchLoc.y - 1 + r, matchLoc.x - 1 + c);
      }
    }
    niftk::InterpolateMaximumOfQuadraticSurface(surface, interpolatedPoint);

    p.id = id;
    p.point.x = rect.x + matchLoc.x + interpolatedPoint.x + halfTemplateSize.x;
    p.point.y = rect.y + matchLoc.y + interpolatedPoint.y + halfTemplateSize.y;

    updatedPoints.insert(niftk::IdPoint2D(id, p));
  }
  return updatedPoints;
}

} // end namespace
