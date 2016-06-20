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
#include <niftkIOUtilities.h>
#include <niftkPointUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Extract AprilTags points", "[AprilTags]" ) {

  if (niftk::argc != 9 && niftk::argc != 10)
  {
    std::cerr << "Usage: niftkExtractAprilTagsPointsTest image tagFamily scaleX scaleY expectedImageWidth expectedImageHeight expectedNumberTags expectIntegerLocations [outputFile]" << std::endl;
    REQUIRE( niftk::argc >= 9);
    REQUIRE( niftk::argc <= 10);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  std::string tagFamily = niftk::argv[2];
  int scaleX = atoi(niftk::argv[3]);
  int scaleY = atoi(niftk::argv[4]);
  int expectedWidth = atoi(niftk::argv[5]);
  int expectedHeight = atoi(niftk::argv[6]);
  int expectedNumberOfTags = atoi(niftk::argv[7]);
  int expectIntegerLocations = atoi(niftk::argv[8]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Point2d scaleFactors;
  scaleFactors.x = scaleX;
  scaleFactors.y = scaleY;

  cv::Mat greyImage;
  cv::cvtColor(image, greyImage, CV_BGR2GRAY);

  niftk::AprilTagsPointDetector detector(false, // don't output corners.
                                         tagFamily,
                                         0,
                                         0.8
                                        );
  detector.SetImage(&greyImage);
  detector.SetImageScaleFactor(scaleFactors);

  niftk::PointSet points;

  if (expectedNumberOfTags == 0)
  {
    REQUIRE_NOTHROW(points = detector.GetPoints());
  }
  else
  {
    points = detector.GetPoints();
  }
  if (expectedNumberOfTags >= 0)
  {
    REQUIRE( points.size() == expectedNumberOfTags );
  }

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

  std::cout << "niftkExtractAprilTagsPointsTest: " << points.size() << std::endl;

  if (niftk::argc >= 10 && points.size() > 0)
  {
    std::string outputFile = niftk::argv[9];
    niftk::SavePointSet(points, outputFile);
  }
}
