/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkCalibrateTimingLag_h
#define niftkCalibrateTimingLag_h

#include "niftkWin32ExportHeader.h"
#include <cv.h>
#include <list>

namespace niftk
{

/**
 * \brief Calibrate timing offset between a point extracted from video frames,
 * and the corresponding tracker matrices.
 *
 * For example, you move a tracked laparoscope up and down, rythmically, while recording
 * a picture of a cross hair. You grab snapshots of the image, and data from
 * a tracker tracking the movement of the laparoscope. If we assume the tracker
 * has less lag, it should be more accurate. So data comes from the video source,
 * and it is timestamped. However, if there is video lag, the timestamp for each video
 * frame will be late, meaning the data was grabbed before the timestamp suggests.
 * So, we need to subtract some time from the timestamp on the video. This function
 * calculates that time offset. The offset is additive, meaning it should be added
 * to the actual timestamp associated with any given video frame.
 */
NIFTYCAL_WINEXPORT double CalibrateTimingOffset(
    std::list< std::pair<cv::Point2d, cv::Matx44d> >
    );

} // end namespace

#endif
