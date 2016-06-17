/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkCirclesIterativePointDetector_h
#define niftkCirclesIterativePointDetector_h

#include "niftkWin32ExportHeader.h"
#include "niftkTemplateMatchingPointDetector.h"

namespace niftk
{

/**
* \class CirclesIterativePointDetector
* \brief Detects circle pattern in a grey scale image, as seen in Dutta 2009
* http://dx.doi.org/10.1109/ICCVW.2009.5457474
*
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT CirclesIterativePointDetector : public TemplateMatchingPointDetector
{

public:

  CirclesIterativePointDetector(cv::Size2i patternSize,      // how many rings in x,y.
                                cv::Size2i offsetForTemplate // how many pixels to search in x,y.
                               );
  virtual ~CirclesIterativePointDetector();

protected:

  /**
  * \see TemplateMatchingPointDetector::GetPointsUsingContours()
  */
  virtual PointSet GetPointsUsingContours(const cv::Mat& image);

};

} // end namespace

#endif
