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
#include <niftkAprilTagsPointDetector.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Extract AprilTags points", "[AprilTags]" ) {

  int expectedNumberOfArguments =  5;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkExtractAprilTagsPointsTest image expectedImageWidth expectedImageHeight expectedNumberTags" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int expectedWidth = atoi(niftk::argv[2]);
  int expectedHeight = atoi(niftk::argv[3]);
  int expectedNumberTags = atoi(niftk::argv[4]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Mat greyImage;
  cv::cvtColor(image, greyImage, CV_BGR2GRAY);

  niftk::AprilTagsPointDetector detector1(&greyImage,
                                         false, // don't include corners
                                         "36h11",
                                         0,
                                         0.8
                                         );
  niftk::PointSet points = detector1.GetPoints();
  REQUIRE( points.size() == expectedNumberTags);

  niftk::AprilTagsPointDetector detector2(&greyImage,
                                         true, // do include corners
                                         "36h11",
                                         0,
                                         0.8
                                         );
  points = detector2.GetPoints();
  REQUIRE( points.size() == expectedNumberTags*5);
}
