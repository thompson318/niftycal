/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkTemplateCirclesPointDetector_h
#define niftkTemplateCirclesPointDetector_h

#include <niftkWin32ExportHeader.h>
#include <niftkTemplateMatchingPointDetector.h>

namespace niftk
{

/**
* \class TemplateCirclesPointDetector
* \brief Detects circle pattern in a grey scale image using Template Matching, as seen in
* <a href="http://dx.doi.org/10.1109/ICCVW.2009.5457474">Datta 2009</a>.
*
* This detector is not thread safe.
*
* \ingroup detectors
*/
class NIFTYCAL_WINEXPORT TemplateCirclesPointDetector : public TemplateMatchingPointDetector
{

public:

  TemplateCirclesPointDetector(cv::Size2i patternSize,       // how many rings in x,y.
                               cv::Size2i offsetForTemplate, // how many pixels to search in x,y.
                               int flags // e.g. = cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING
                              );
  virtual ~TemplateCirclesPointDetector();

protected:

  /**
  * \see TemplateMatchingPointDetector::GetPointsUsingContours()
  */
  virtual PointSet GetPointsUsingContours(const cv::Mat& image);

};

} // end namespace

#endif
