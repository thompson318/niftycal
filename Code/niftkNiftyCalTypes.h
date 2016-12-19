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
#include <vector>

/**
* \defgroup internal internal
* \brief Internal stuff, not for end-users.
*/

/**
* \defgroup types types
* \brief Package-wide data types.
*/

/**
* \defgroup utilities utilities
* \brief Groups of c-style functions.
*/

/**
* \defgroup detectors detectors
* \brief A suite of classes for point detection in images.
*/

/**
* \defgroup calibration calibration
* \brief The main calibration routines.
*/

/**
* \defgroup applications applications
* \brief Small, end-user applications, most likely for testing.
*/

/**
* \file niftkNiftyCalTypes.h
* \brief Defines types used in this library.
* \ingroup types
*/

//! Single namespace for all code in this package
namespace niftk
{

typedef unsigned int NiftyCalIdType;

/**
* \brief Clock time in nano-seconds since epoch.
*/
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

struct NIFTYCAL_WINEXPORT TimingSample1D
{
  NiftyCalTimeType time;
  double           sample;
};

struct NIFTYCAL_WINEXPORT TimingSample2D
{
  NiftyCalTimeType time;
  cv::Point2d      sample;
};

struct NIFTYCAL_WINEXPORT TimingSample3D
{
  NiftyCalTimeType time;
  cv::Point3d      sample;
};

typedef std::map <NiftyCalIdType, Point2D>   PointSet;
typedef std::pair<NiftyCalIdType, Point2D>   IdPoint2D;
typedef std::map <NiftyCalIdType, Point3D>   Model3D;
typedef std::pair<NiftyCalIdType, Point3D>   IdPoint3D;
typedef std::vector<TimingSample1D>          TimeSamples1D;
typedef std::vector<TimingSample2D>          TimeSamples2D;
typedef std::vector<TimingSample3D>          TimeSamples3D;
typedef std::map<NiftyCalTimeType, double>   TimeMappedSamples1D;

} // end namespace

#endif
