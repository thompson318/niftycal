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
#include <niftkOpenCVChessboardPointDetector.h>

TEST_CASE( "Extract chessboard points", "[chessboard]" ) {

  int expectedNumberOfArguments =  6;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkExtractChessboardPoints image expectedImageWidth expectedImageHeight expectedNumberInternalCornersX expectedNumberInternalCornersY" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int expectedWidth = atoi(niftk::argv[2]);
  int expectedHeight = atoi(niftk::argv[3]);
  int expectedInternalCornersX = atoi(niftk::argv[4]);
  int expectedInternalCornersY = atoi(niftk::argv[5]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Size2i internalCorners(expectedInternalCornersX, expectedInternalCornersY);

  REQUIRE_THROWS(niftk::OpenCVChessboardPointDetector failingDetector(NULL, internalCorners));

  niftk::OpenCVChessboardPointDetector detector(&image, internalCorners);
  niftk::PointSet points = detector.GetPoints();

  REQUIRE( points.size() == expectedInternalCornersX * expectedInternalCornersY );
}
