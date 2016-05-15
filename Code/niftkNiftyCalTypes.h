/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNiftyCalTypes_h
#define niftkNiftyCalTypes_h

#include "niftkWin32ExportHeader.h"
#include <cv.h>
#include <map>

/**
* \file niftkTypes.h
* \brief Defines types used in this library.
*/
namespace niftk
{

typedef unsigned int NiftyCalIdType;
typedef unsigned long long NiftyCalTimeType;

/**
* \class Point2D
* \brief Placeholder for a 2D point and its associated identifier.
*/
struct NIFTYCAL_WINEXPORT Point2D
{
  NiftyCalIdType id;
  cv::Point2d    point;
};

/**
* \class Point3D
* \brief Placeholder for a 3D point and its associated identifier.
*/
struct NIFTYCAL_WINEXPORT Point3D
{
  NiftyCalIdType id;
  cv::Point3d    point;
};

typedef std::map <NiftyCalIdType, Point2D> PointSet;
typedef std::pair<NiftyCalIdType, Point2D> IdPoint2D;
typedef std::map <NiftyCalIdType, Point3D> Model3D;
typedef std::pair<NiftyCalIdType, Point3D> IdPoint3D;

} // end namespace

#endif
