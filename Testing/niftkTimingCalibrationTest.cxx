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
#include <niftkTimingCalibration.h>
#include <niftkIOUtilities.h>

TEST_CASE( "Timing Calibration 2D/3D", "[timing]" ) {

  int expectedNumberOfArguments =  4;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkTimingCalibrationTest trackerPoints3D.txt imagePoints2D.txt expectedLagInMilliseconds" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  std::string trackerPointsFileName = niftk::argv[1];
  REQUIRE(trackerPointsFileName.size() > 0);

  std::string imagePointsFileName = niftk::argv[2];
  REQUIRE(imagePointsFileName.size() > 0);

  int expectedLag = atoi(niftk::argv[3]);

  niftk::TimeSamples3D time3D = niftk::LoadTimeSamples3D(trackerPointsFileName);
  REQUIRE(time3D.size() > 0);

  niftk::TimeSamples2D time2D = niftk::LoadTimeSamples2D(imagePointsFileName);
  REQUIRE(time2D.size() > 0);

  int actualLag = niftk::TimingCalibration(time3D, time2D);
  REQUIRE(expectedLag == actualLag);
}
