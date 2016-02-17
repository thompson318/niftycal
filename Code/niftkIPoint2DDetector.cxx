/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIPoint2DDetector.h"

namespace niftk {

//-----------------------------------------------------------------------------
PointSet CopyPoints(const PointSet& p)
{
  PointSet result;

  PointSet::const_iterator iter;
  for ( iter = p.begin(); iter != p.end(); ++iter )
  {
    result.insert(IdPoint2D((*iter).first, (*iter).second));
  }

  return result;
}


//-----------------------------------------------------------------------------
PointSet RescalePoints(const PointSet& p, const cv::Point2d& scaleFactor)
{
  PointSet result;

  PointSet::const_iterator iter;
  for ( iter = p.begin(); iter != p.end(); ++iter )
  {
    Point2D tmp;
    tmp.point = (*iter).second.point;
    tmp.id = (*iter).second.id;
    tmp.point.x *= scaleFactor.x;
    tmp.point.y *= scaleFactor.y;
    result.insert(IdPoint2D(tmp.id, tmp));
  }

  return result;
}


//-----------------------------------------------------------------------------
IPoint2DDetector::IPoint2DDetector()
{

}


//-----------------------------------------------------------------------------
IPoint2DDetector::~IPoint2DDetector()
{

}

} // end namespace
