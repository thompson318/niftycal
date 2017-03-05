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
#include <niftkIOUtilities.h>
#include <niftkPointUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Extract assymetric circle points", "[circles]" ) {

  if (niftk::argc != 11 && niftk::argc != 12)
  {
    std::cerr << "Usage: niftkExtractCirclesPointsTest image scaleX scaleY expectedImageWidth expectedImageHeight expectedColumns expectedCirclesPerColumn expectedNumberOfCircles expectIntegerLocations asymmetric [outputFile]" << std::endl;
    REQUIRE( niftk::argc >= 11);
    REQUIRE( niftk::argc <= 12);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int scaleX = atoi(niftk::argv[2]);
  int scaleY = atoi(niftk::argv[3]);
  int expectedWidth = atoi(niftk::argv[4]);
  int expectedHeight = atoi(niftk::argv[5]);
  int expectedColumns = atoi(niftk::argv[6]);
  int expectedCirclesPerColumn = atoi(niftk::argv[7]);
  int expectedNumberOfCircles = atoi(niftk::argv[8]);
  int expectIntegerLocations = atoi(niftk::argv[9]);
  int asymmetric = atoi(niftk::argv[10]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Point2d scaleFactors;
  scaleFactors.x = scaleX;
  scaleFactors.y = scaleY;

  cv::Mat greyImage;
  cv::cvtColor(image, greyImage, CV_BGR2GRAY);

  int flags = cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING;
  if (asymmetric == 1)
  {
    flags = cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING;
  }

  cv::Size2i patternSize(expectedColumns, expectedCirclesPerColumn);
  niftk::CirclesPointDetector detector(patternSize, flags);
  detector.SetCaching(true);
  detector.SetImage(&greyImage);
  detector.SetImageScaleFactor(scaleFactors);

  niftk::PointSet points;
  if (expectedNumberOfCircles < 0)
  {
    REQUIRE_NOTHROW(points = detector.GetPoints());
  }
  else
  {
    points = detector.GetPoints();
    REQUIRE( points.size() == expectedNumberOfCircles );
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

  if (niftk::argc == 12 && points.size() > 0)
  {
    std::string outputFile = niftk::argv[11];
    niftk::SavePointSet(points, outputFile);
  }
}
