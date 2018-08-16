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
#include "niftkNiftyCalTypes.h"
#include "niftkIOUtilities.h"

TEST_CASE( "Load 3D Model, check against expected number of points", "[IO]" ) {

  int expectedNumberOfArguments = 3;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkLoadModelTest model.txt expectedNumberOfPoints" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  niftk::Model3D model = niftk::LoadModel3D(niftk::argv[1]);

  int expectedNumberOfPoints = atoi(niftk::argv[2]);
  REQUIRE(model.size() == expectedNumberOfPoints);
}
