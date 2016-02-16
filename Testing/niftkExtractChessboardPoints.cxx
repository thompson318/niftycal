/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#define CATCH_CONFIG_RUNNER  // This tells Catch we provide main.
#include "catch.hpp"

#include <cv.h>
#include <highgui.h>
#include <stdlib.h>
#include <niftkIPointDetector.h>
#include <niftkOpenCVChessboardPointDetector.h>

cv::Mat image;
int expectedWidth(0);
int expectedHeight(0);
int expectedInternalCornersX(0);
int expectedInternalCornersY(0);

TEST_CASE( "Extract chessboard points", "[chessboard]" ) {

  REQUIRE( image.rows == expectedWidth );
  REQUIRE( image.cols == expectedHeight );

  cv::Size2i internalCorners(expectedInternalCornersX, expectedInternalCornersY);

  niftk::OpenCVChessboardPointDetector detector(&image, internalCorners);
  std::vector< niftk::Point2D > points = detector.GetPoints();

  REQUIRE( points.size() == expectedInternalCornersX * expectedInternalCornersY );
}

int main (int argc, char * const argv[])
{
  if (argc != 6)
  {
    std::cerr << "Usage: niftkExtractChessboardPoints image expectedWidth expectedHeight expectedInternalCornersX expectedInternalCornersY" << std::endl;
    return EXIT_FAILURE;
  }

  Catch::Session session; // There must be exactly once instance
  int returnCode = session.applyCommandLine(argc, argv);
  if(returnCode != 0) // Indicates a command line error
    return returnCode;
  image = cv::imread(argv[1]);
  expectedWidth = atoi(argv[2]);
  expectedHeight = atoi(argv[3]);
  expectedInternalCornersX = atoi(argv[4]);
  expectedInternalCornersY = atoi(argv[5]);
  return session.run();
}
