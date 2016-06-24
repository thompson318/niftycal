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
#include <niftkPointUtilities.h>

#include <cv.h>

TEST_CASE( "Distance test 2D", "[points]" ) {

  cv::Point2d a(1, 2);
  REQUIRE(niftk::DistanceBetween(a, a) == 0);

  cv::Point2d b(1, 2);
  REQUIRE(niftk::DistanceBetween(a, b) == 0);

  b.x = 2;
  REQUIRE(niftk::DistanceBetween(a, b) == 1);

  b.x = 1;
  b.y = 3;
  REQUIRE(niftk::DistanceBetween(a, b) == 1);

  b.x = 2;
  REQUIRE(niftk::DistanceBetween(a, b) == std::sqrt(2));
}


TEST_CASE( "Distance test 3D", "[points]" ) {

  cv::Point3d a(1, 2, 0);
  REQUIRE(niftk::DistanceBetween(a, a) == 0);

  cv::Point3d b(1, 2, 0);
  REQUIRE(niftk::DistanceBetween(a, b) == 0);

  b.x = 2;
  REQUIRE(niftk::DistanceBetween(a, b) == 1);

  b.x = 1;
  b.y = 3;
  REQUIRE(niftk::DistanceBetween(a, b) == 1);

  b.y = 2;
  b.z = 1;
  REQUIRE(niftk::DistanceBetween(a, b) == 1);

  b.x = 2;
  b.y = 3;
  REQUIRE(niftk::DistanceBetween(a, b) == std::sqrt(3));
}
