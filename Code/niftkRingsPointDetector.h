/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkRingsPointDetector_h
#define niftkRingsPointDetector_h

#include "niftkWin32ExportHeader.h"
#include "niftkTemplateMatchingPointDetector.h"

namespace niftk
{

/**
* \class RingsPointDetector
* \brief Detects rings pattern, as seen in
* <a href="http://dx.doi.org/10.1109/ICCVW.2009.5457474">Dutta 2009</a>.
*
* Our method for finding blobs is:
* <ol>
*   <li>Threshold using cv::threshold and m_ThresholdValue which defaults to 50.
*       This can be changed via the SetThreshold() method.</li>
*   <li>Use cv::findContours() to find inner and outer contours.
*   <li>If the full set of contours is not found, try adaptive thresholding using
*       cv::adaptiveThreshold() and ADAPTIVE_THRESH_MEAN_C, and m_AdaptiveThreshold which
*       defaults to 75. This can also be changed with SetAdaptiveThreshold().</li>
*   <li>If the full set of contours is not found, give up. </li>
*   <li>Convert the inner contours to an image of little blobs. </li>
*   <li>Convert the outer contours to an image of big blobs. </li>
*   <li>Use cv::findCirclesGrid() using cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING
*       on both little and big blobs</li>
*   <li>Take the average position of each pair of little/big blobs.
* </ol>
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT RingsPointDetector : public TemplateMatchingPointDetector
{

public:

  RingsPointDetector(cv::Size2i patternSize,      // how many rings in x,y.
                     cv::Size2i offsetForTemplate // how many pixels to search in x,y.
                    );
  virtual ~RingsPointDetector();
  void SetUseOuterContour(const bool& useIt);     // Default to true.
  void SetThreshold(const unsigned char& thresholdValue); // Default to 50.
  void SetAdaptiveThreshold(const unsigned char& thresholdValue); // Default to 75;

protected:

  /**
  * \see TemplateMatchingPointDetector::GetPointsUsingContours()
  */
  virtual PointSet GetPointsUsingContours(const cv::Mat& image);

private:

  void ExtractBlobs(const cv::Mat& image, cv::Mat& bigBlobs, cv::Mat& littleBlobs);
  void ExtractIndexes(const cv::Mat& image,
                      std::vector<cv::Vec4i>& hierarchy,
                      std::vector<std::vector<cv::Point> >& contours,
                      std::vector<unsigned int>& inner,
                      std::vector<unsigned int>& outer);

  bool          m_UseOuterContour;
  unsigned char m_ThresholdValue;
  unsigned char m_AdaptiveThreshold;
};

} // end namespace

#endif
