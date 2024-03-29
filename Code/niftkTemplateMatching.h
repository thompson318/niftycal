/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkTemplateMatching_h
#define niftkTemplateMatching_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"

namespace niftk
{

/**
* \file niftkTemplateMatching.h
* \brief Matches the template to the image around each starting point,
* shifting the template window in a region given by the specified offset.
* \ingroup utilities
*/
NIFTYCAL_WINEXPORT PointSet DoTemplateMatchingForAllPoints(const cv::Mat& image,
                                                           const cv::Mat& templateImage,
                                                           const cv::Size2i& offset,
                                                           const niftk::PointSet& startingPoints
                                                          );
} // end namespace

#endif
