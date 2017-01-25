/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkSideBySideDetector_h
#define niftkSideBySideDetector_h

#include <niftkWin32ExportHeader.h>
#include <niftkPointDetector.h>
#include <memory>

namespace niftk
{

/**
* \class SideBySideDetector
* \brief Runs two detectors on left and right hand side of a given image.
*
* This detector is not thread safe.
*
* \ingroup detectors
*/
class NIFTYCAL_WINEXPORT SideBySideDetector : public PointDetector
{
public:

  /** Constructor takes ownership. */
  SideBySideDetector(std::unique_ptr<niftk::PointDetector>& left,
                     std::unique_ptr<niftk::PointDetector>& right
                     );
  virtual ~SideBySideDetector();

protected:

  /**
  * \see niftk::PointDetector::InternalGetPoints()
  */
  virtual PointSet InternalGetPoints(const cv::Mat& imageToUse);

private:

  std::unique_ptr<niftk::PointDetector> m_LeftDetector;
  std::unique_ptr<niftk::PointDetector> m_RightDetector;
};

} // end namespace

#endif
