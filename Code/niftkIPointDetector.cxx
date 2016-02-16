/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIPointDetector.h"

namespace niftk {

//-----------------------------------------------------------------------------
std::vector<Point2D> CopyPoints(const std::vector<Point2D>& p)
{
  std::vector<Point2D> result;

  if ( p.size() > 0 )
  {
    std::vector<Point2D>::const_iterator iter;
    for ( iter = p.begin(); iter != p.end(); ++iter )
    {
      result.push_back(*iter);
    }
  }
  return result;
}


//-----------------------------------------------------------------------------
std::vector<Point2D> RescalePoints(const std::vector<Point2D>& p, const cv::Point2d& scaleFactor)
{
  std::vector<Point2D> result;
  if ( result.size() > 0 )
  {
    std::vector<Point2D>::const_iterator iter;
    for ( iter = p.begin(); iter != p.end(); ++iter )
    {
      Point2D tmp;
      tmp.point = (*iter).point;
      tmp.id = (*iter).id;
      tmp.point.x *= scaleFactor.x;
      tmp.point.y *= scaleFactor.y;
      result.push_back(tmp);
    }
  }
  return result;
}


//-----------------------------------------------------------------------------
IPointDetector::IPointDetector()
{

}


//-----------------------------------------------------------------------------
IPointDetector::~IPointDetector()
{

}

} // end namespace
