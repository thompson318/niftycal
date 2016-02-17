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
#include <map>

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

typedef std::map<unsigned int, Point2D> PointSet;
typedef std::pair<unsigned int, Point2D> IdPoint;

/**
* \brief Creates a new copy of the input list.
*/
NIFTYCAL_WINEXPORT PointSet CopyPoints(const PointSet& p);


/**
* \brief Rescales by multiplying each point by the scale factor.
* \param scaleFactor contains a multiplier for x,y.
*/
NIFTYCAL_WINEXPORT PointSet RescalePoints(const PointSet& p, const cv::Point2d& scaleFactor);


/**
* \class IPoint2DDetector
* \brief Interface for anything that detects 2D points in an image.
*
* The point here, is that implementing classes should *just* detect
* the points. There is no conversion to millimetres, no camera
* calibration, nothing but point detection. The points can be
* chessboard corners, centres of circles, SIFT points etc.
*/
class NIFTYCAL_WINEXPORT IPoint2DDetector
{

public:

  IPoint2DDetector();
  virtual ~IPoint2DDetector();

  /**
  * \brief Retrieves points, each one identified by a Point2D.
  * \return PointSet of Point2D.
  */
  virtual PointSet GetPoints() = 0;

};

} // end namespace

#endif
