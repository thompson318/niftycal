/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkIPointDetector_h
#define niftkIPointDetector_h

#include "niftkWin32ExportHeader.h"
#include <cv.h>

namespace niftk
{

/**
* \class PointInfo
* \brief Placeholder for a point and its associated identifier.
*/
struct NIFTYCAL_WINEXPORT Point2D
{
  unsigned int id;
  cv::Point2d point;
};


/**
* \brief Creates a new copy of the input list.
*/
NIFTYCAL_WINEXPORT std::vector<Point2D> CopyPoints(const std::vector<Point2D>& p);


/**
* \brief Rescales by multiplying each point by the scale factor.
* \param scaleFactor contains a multiplier for x,y.
*/
NIFTYCAL_WINEXPORT std::vector<Point2D> RescalePoints(const std::vector<Point2D>& p, const cv::Point2d& scaleFactor);


/**
* \class IPointDetector
* \brief Interface for anything that detects points in an image.
*/
class NIFTYCAL_WINEXPORT IPointDetector
{

public:

  IPointDetector();
  virtual ~IPointDetector();

  /**
  * \brief Retrieves points, each one identified by a Point2D.
  * \return vector of Point2D.
  */
  virtual std::vector< Point2D > GetPoints() = 0;

};

} // end namespace

#endif
