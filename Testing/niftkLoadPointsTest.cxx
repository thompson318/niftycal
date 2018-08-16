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
#include <niftkNiftyCalTypes.h>
#include <niftkIOUtilities.h>
#include <niftkPointUtilities.h>

TEST_CASE( "Load 2D Points, check against expected number of points", "[IO]" ) {

  int expectedNumberOfArguments = 3;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkLoadPointsTest points.txt expectedNumberOfPoints" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  niftk::PointSet points = niftk::LoadPointSet(niftk::argv[1]);

  int expectedNumberOfPoints = atoi(niftk::argv[2]);
  REQUIRE(points.size() == expectedNumberOfPoints);

  niftk::DumpPoints(std::cout, points);
}
