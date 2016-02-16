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

#include <cv.h>
#include <highgui.h>
#include <niftkOpenCVChessboardPointDetector.h>

TEST_CASE( "Extract chessboard points", "[chessboard]" ) {

  if (niftk::argc != 6)
  {
    std::cerr << "Usage: niftkExtractChessboardPoints image expectedImageWidth expectedImageHeight expectedNumberInternalCornersX expectedNumberInternalCornersY" << std::endl;
    REQUIRE( niftk::argc == 6);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int expectedWidth = atoi(niftk::argv[2]);
  int expectedHeight = atoi(niftk::argv[3]);
  int expectedInternalCornersX = atoi(niftk::argv[4]);
  int expectedInternalCornersY = atoi(niftk::argv[5]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Size2i internalCorners(expectedInternalCornersX, expectedInternalCornersY);

  niftk::OpenCVChessboardPointDetector detector(&image, internalCorners);
  std::vector< niftk::Point2D > points = detector.GetPoints();

  REQUIRE( points.size() == expectedInternalCornersX * expectedInternalCornersY );
}
