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
#include <niftkOpenCVCirclesPointDetector.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Extract assymetric circle points", "[circles]" ) {

  int expectedNumberOfArguments =  6;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkExtractCirclesPointsTest image expectedImageWidth expectedImageHeight expectedColumns expectedCirclesPerColumn " << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int expectedWidth = atoi(niftk::argv[2]);
  int expectedHeight = atoi(niftk::argv[3]);
  int expectedColumns = atoi(niftk::argv[4]);
  int expectedCirclesPerColumn = atoi(niftk::argv[5]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Mat greyImage;
  cv::cvtColor(image, greyImage, CV_BGR2GRAY);

  cv::Size2i patternSize(expectedCirclesPerColumn, expectedColumns);
  niftk::OpenCVCirclesPointDetector detector(patternSize);
  detector.SetImage(&greyImage);
  niftk::PointSet points = detector.GetPoints();

  REQUIRE( points.size() == expectedCirclesPerColumn * expectedColumns );
}
