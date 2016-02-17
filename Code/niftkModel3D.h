/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkModel3D_h
#define niftkModel3D_h

#include "niftkWin32ExportHeader.h"
#include <cv.h>
#include <map>

namespace niftk
{

/**
* \class Point3D
* \brief Placeholder for a 3D point and its associated identifier.
*/
struct NIFTYCAL_WINEXPORT Point3D
{
  unsigned int id;
  cv::Point3d point;
};

typedef std::map<unsigned int, Point3D> Model3D;
typedef std::pair<unsigned int, Point3D> IdPoint3D;

} // end namespace

#endif
