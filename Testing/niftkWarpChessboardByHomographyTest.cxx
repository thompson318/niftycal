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
#include <niftkHomographyUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Warp chessboard to match another", "[chessboard]" ) {

  int expectedNumberOfArguments =  6;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkWarpChessboardByHomographyTest image1 image2 intCornersX intCornersY expectedRMS" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  cv::Mat image1 = cv::imread(niftk::argv[1]);
  cv::Mat image2 = cv::imread(niftk::argv[2]);
  int numberInternalCornersInX = atoi(niftk::argv[3]);
  int numberInternalCornersInY = atoi(niftk::argv[4]);
  float expectedRMS = atof(niftk::argv[5]);

  cv::Mat greyImage1;
  cv::cvtColor(image1, greyImage1, CV_BGR2GRAY);

  cv::Mat greyImage2;
  cv::cvtColor(image2, greyImage2, CV_BGR2GRAY);

  cv::Mat warpedImage = greyImage1;
  cv::Size2i corners(numberInternalCornersInX, numberInternalCornersInY);

  niftk::OpenCVChessboardPointDetector sourceDetector(&greyImage1, corners);
  niftk::PointSet sourcePointSet = sourceDetector.GetPoints();
  REQUIRE(sourcePointSet.size() >= 4);

  niftk::OpenCVChessboardPointDetector targetDetector(&greyImage2, corners);
  niftk::PointSet targetPointSet = targetDetector.GetPoints();
  REQUIRE(targetPointSet.size() >= 4);
  REQUIRE(targetPointSet.size() == sourcePointSet.size());

  niftk::WarpImageByCorrespondingPoints(greyImage1, sourcePointSet, targetPointSet, greyImage1.size(), warpedImage);
  REQUIRE(greyImage2.rows == warpedImage.rows);
  REQUIRE(greyImage2.cols == warpedImage.cols);
}
