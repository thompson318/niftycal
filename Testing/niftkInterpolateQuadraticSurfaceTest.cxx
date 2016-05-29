/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "catch.hpp"
#include "niftkCatchMain.h"
#include <niftkMatrixUtilities.h>

#include <cv.h>

namespace niftk
{
  void makeQuadratic(cv::Matx33d& matrix, const cv::Point2d& offset)
  {
    for (int y = -1; y <= 1; y++)
    {
      for (int x = -1; x <= 1; x++)
      {
        matrix(y+1, x+1) = -(  (x - offset.x) * (x - offset.x)
                             + (y - offset.y) * (y - offset.y)
                            );
      }
    }
  }
}

TEST_CASE( "Interpolate Surface", "[interpolation]" ) {

  int expectedNumberOfArguments =  1;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkInterpolateQuadraticSurfaceTest" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  cv::Point2d offset;
  cv::Point2d point;
  cv::Matx33d matrix;

  double tolerance = 0.001;
  offset.x = 0.1;
  offset.y = 0.2;
  niftk::makeQuadratic(matrix, offset);
  niftk::InterpolateMaximumOfQuadraticSurface(matrix, point);
  REQUIRE(fabs(point.x - offset.x) < tolerance);
  REQUIRE(fabs(point.y - offset.y) < tolerance);

  offset.x = -0.4;
  offset.y =  0.2;
  niftk::makeQuadratic(matrix, offset);
  niftk::InterpolateMaximumOfQuadraticSurface(matrix, point);
  REQUIRE(fabs(point.x - offset.x) < tolerance);
  REQUIRE(fabs(point.y - offset.y) < tolerance);
}
