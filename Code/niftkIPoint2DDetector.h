/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkIPoint2DDetector_h
#define niftkIPoint2DDetector_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"

namespace niftk
{

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
