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
#include <niftkOpenCVChessboardPointDetector.h>
#include <niftkIOUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Extract chessboard points", "[chessboard]" ) {

  if (niftk::argc != 8 && niftk::argc != 9)
  {
    std::cerr << "Usage: niftkExtractChessboardPointsTest image scaleX scaleY expectedImageWidth expectedImageHeight expectedNumberInternalCornersX expectedNumberInternalCornersY [outputFile]" << std::endl;
    REQUIRE( niftk::argc >= 8);
    REQUIRE( niftk::argc <= 9);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int scaleX = atoi(niftk::argv[2]);
  int scaleY = atoi(niftk::argv[3]);
  int expectedWidth = atoi(niftk::argv[4]);
  int expectedHeight = atoi(niftk::argv[5]);
  int expectedInternalCornersX = atoi(niftk::argv[6]);
  int expectedInternalCornersY = atoi(niftk::argv[7]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Point2d scaleFactors;
  scaleFactors.x = scaleX;
  scaleFactors.y = scaleY;

  cv::Mat greyImage;
  cv::cvtColor(image, greyImage, CV_BGR2GRAY);

  cv::Size2i tooFewInternalCornersWidth(1, 2);
  REQUIRE_THROWS(niftk::OpenCVChessboardPointDetector failingDetector1(tooFewInternalCornersWidth));

  cv::Size2i tooFewInternalCornersHeight(2, 1);
  REQUIRE_THROWS(niftk::OpenCVChessboardPointDetector failingDetector2(tooFewInternalCornersHeight));

  cv::Size2i internalCorners(expectedInternalCornersX, expectedInternalCornersY);
  niftk::OpenCVChessboardPointDetector detector(internalCorners);
  detector.SetImage(&greyImage);
  detector.SetImageScaleFactor(scaleFactors);

  niftk::PointSet points = detector.GetPoints();
  REQUIRE( points.size() == expectedInternalCornersX * expectedInternalCornersY );

  std::cout << "niftkExtractChessboardPointsTest: " << points.size() << std::endl;

  if (niftk::argc == 9)
  {
    std::string outputFile = niftk::argv[8];
    niftk::SavePointSet(points, outputFile);
  }
}
