/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkTypes_h
#define niftkTypes_h

#include "niftkWin32ExportHeader.h"
#include <cv.h>
#include <map>

/**
* \file niftkTypes.h
* \brief Defines all types used in this library.
*/
namespace niftk
{

typedef unsigned int IdType;

/**
* \class Point2D
* \brief Placeholder for a 2D point and its associated identifier.
*/
struct NIFTYCAL_WINEXPORT Point2D
{
  IdType id;
  cv::Point2d  point;
};

/**
* \class Point3D
* \brief Placeholder for a 3D point and its associated identifier.
*/
struct NIFTYCAL_WINEXPORT Point3D
{
  IdType id;
  cv::Point3d point;
};

typedef std::map<IdType, Point2D> PointSet;
typedef std::pair<IdType, Point2D> IdPoint2D;
typedef std::map<IdType, Point3D> Model3D;
typedef std::pair<IdType, Point3D> IdPoint3D;

/**
* \brief Creates a new copy of the input list.
*/
NIFTYCAL_WINEXPORT PointSet CopyPoints(const PointSet& p);


/**
* \brief Rescales by multiplying each point by the scale factor.
* \param scaleFactor contains a multiplier for x,y.
*/
NIFTYCAL_WINEXPORT PointSet RescalePoints(const PointSet& p, const cv::Point2d& scaleFactor);

} // end namespace

#endif
