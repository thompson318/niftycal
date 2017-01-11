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
#include <niftkCirclesPointDetector.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkPointUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Extract assymetric circle points", "[circles]" ) {

  int expectedNumberOfArguments =  8;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkExtractCirclesPointsTest image expectedImageWidth expectedImageHeight expectedColumns expectedCirclesPerColumn expectedNumberOfCircles expectIntegerLocations" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int expectedWidth = atoi(niftk::argv[2]);
  int expectedHeight = atoi(niftk::argv[3]);
  int expectedColumns = atoi(niftk::argv[4]);
  int expectedCirclesPerColumn = atoi(niftk::argv[5]);
  int expectedNumberOfCircles = atoi(niftk::argv[6]);
  int expectIntegerLocations = atoi(niftk::argv[7]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Mat greyImage;
  cv::cvtColor(image, greyImage, CV_BGR2GRAY);

  cv::Size2i patternSize(expectedCirclesPerColumn, expectedColumns);
  niftk::CirclesPointDetector detector(patternSize, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
  detector.SetImage(&greyImage);

  niftk::PointSet points;
  if (expectedNumberOfCircles == 0)
  {
    REQUIRE_NOTHROW(points = detector.GetPoints());
  }
  else
  {
    points = detector.GetPoints();
    REQUIRE( points.size() == expectedCirclesPerColumn * expectedColumns );
  }
  REQUIRE( points.size() == expectedNumberOfCircles );

  if (points.size() > 0)
  {
    bool containsNonIntegerPoints = niftk::PointSetContainsNonIntegerPositions(points);
    if (expectIntegerLocations == 1 && containsNonIntegerPoints)
    {
      niftkNiftyCalThrow() << "Found non-integer coordinates.";
    }
    else if (expectIntegerLocations != 1 && !containsNonIntegerPoints)
    {
      niftkNiftyCalThrow() << "Did not find non-integer coordinates.";
    }
  }
}
