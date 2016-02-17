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
#include <iostream>

#include <cv.h>
#include <highgui.h>
#include <niftkAprilTagsPointDetector.h>

TEST_CASE( "Extract AprilTags points", "[AprilTags]" ) {

  int expectedNumberOfArguments =  5;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkExtractChessboardPoints image expectedImageWidth expectedImageHeight expectedNumberTags" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int expectedWidth = atoi(niftk::argv[2]);
  int expectedHeight = atoi(niftk::argv[3]);
  int expectedNumberTags = atoi(niftk::argv[4]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  niftk::AprilTagsPointDetector detector(&image);
  niftk::PointSet points = detector.GetPoints();

  REQUIRE( points.size() == expectedNumberTags);

}
