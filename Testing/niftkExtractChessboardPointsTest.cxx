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

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Extract chessboard points", "[chessboard]" ) {

  int expectedNumberOfArguments =  6;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkExtractChessboardPointsTest image expectedImageWidth expectedImageHeight expectedNumberInternalCornersX expectedNumberInternalCornersY" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int expectedWidth = atoi(niftk::argv[2]);
  int expectedHeight = atoi(niftk::argv[3]);
  int expectedInternalCornersX = atoi(niftk::argv[4]);
  int expectedInternalCornersY = atoi(niftk::argv[5]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Mat greyImage;
  cv::cvtColor(image, greyImage, CV_BGR2GRAY);

  cv::Size2i tooFewInternalCornersWidth(1, 2);
  REQUIRE_THROWS(niftk::OpenCVChessboardPointDetector failingDetector1(tooFewInternalCornersWidth));

  cv::Size2i tooFewInternalCornersHeight(2, 1);
  REQUIRE_THROWS(niftk::OpenCVChessboardPointDetector failingDetector2(tooFewInternalCornersHeight));

  cv::Size2i internalCorners(expectedInternalCornersX, expectedInternalCornersY);
  niftk::OpenCVChessboardPointDetector detector(internalCorners);
  detector.SetImage(&greyImage);
  niftk::PointSet points = detector.GetPoints();

  REQUIRE( points.size() == expectedInternalCornersX * expectedInternalCornersY );
}
