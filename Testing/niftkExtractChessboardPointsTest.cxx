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
#include <niftkChessboardPointDetector.h>
#include <niftkIOUtilities.h>
#include <niftkPointUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <chrono>

TEST_CASE( "Extract chessboard points", "[chessboard]" ) {

  if (niftk::argc != 10 && niftk::argc != 11)
  {
    std::cerr << "Usage: niftkExtractChessboardPointsTest image scaleX scaleY expectedImageWidth expectedImageHeight expectedNumberInternalCornersX expectedNumberInternalCornersY expectedNumberOfCorners expectIntegerLocations [outputFile]" << std::endl;
    REQUIRE( niftk::argc >= 10);
    REQUIRE( niftk::argc <= 11);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int scaleX = atoi(niftk::argv[2]);
  int scaleY = atoi(niftk::argv[3]);
  int expectedWidth = atoi(niftk::argv[4]);
  int expectedHeight = atoi(niftk::argv[5]);
  int expectedInternalCornersX = atoi(niftk::argv[6]);
  int expectedInternalCornersY = atoi(niftk::argv[7]);
  int expectedNumberOfCorners = atoi(niftk::argv[8]);
  int expectIntegerLocations = atoi(niftk::argv[9]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Point2d scaleFactors;
  scaleFactors.x = scaleX;
  scaleFactors.y = scaleY;

  cv::Mat greyImage;
  cv::cvtColor(image, greyImage, CV_BGR2GRAY);

  cv::Size2i tooFewInternalCornersWidth(1, 2);
  REQUIRE_THROWS(niftk::ChessboardPointDetector failingDetector1(tooFewInternalCornersWidth));

  cv::Size2i tooFewInternalCornersHeight(2, 1);
  REQUIRE_THROWS(niftk::ChessboardPointDetector failingDetector2(tooFewInternalCornersHeight));

  cv::Size2i internalCorners(expectedInternalCornersX, expectedInternalCornersY);
  niftk::ChessboardPointDetector detector(internalCorners);
  detector.SetCaching(true);
  detector.SetImage(&greyImage);
  detector.SetImageScaleFactor(scaleFactors);

  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();

  niftk::PointSet points;

  if (expectedNumberOfCorners == 0)
  {
    REQUIRE_NOTHROW(points = detector.GetPoints());
  }
  else
  {
    points = detector.GetPoints();
    REQUIRE( points.size() == expectedInternalCornersX * expectedInternalCornersY );
  }
  REQUIRE( points.size() == expectedNumberOfCorners );

  std::chrono::time_point<std::chrono::system_clock> endFirstGrab = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = endFirstGrab - start;
  std::cout << "TIME:First grab=" << elapsed_seconds.count() << std::endl;

  points = detector.GetPoints();

  std::chrono::time_point<std::chrono::system_clock> endSecondGrab = std::chrono::system_clock::now();
  elapsed_seconds = endSecondGrab - endFirstGrab;
  std::cout << "TIME:Second grab=" << elapsed_seconds.count() << std::endl;

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

  if (niftk::argc == 11 && points.size() > 0)
  {
    std::string outputFile = niftk::argv[10];
    niftk::SavePointSet(points, outputFile);
  }
}
