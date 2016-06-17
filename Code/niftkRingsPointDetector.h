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

protected:

  /**
  * \see TemplateMatchingPointDetector::GetPointsUsingContours()
  */
  virtual PointSet GetPointsUsingContours(const cv::Mat& image);

private:

  void ExtractBlobs(const cv::Mat& image, cv::Mat& bigBlobs, cv::Mat& littleBlobs);
  bool m_UseOuterContour;
};

} // end namespace

#endif
