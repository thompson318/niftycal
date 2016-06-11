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
#include <niftkPointUtilities.h>
#include <niftkHomographyUtilities.h>
#include <niftkChessboardPointDetector.h>

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

  niftk::ChessboardPointDetector sourceDetector(corners);
  sourceDetector.SetImage(&greyImage1);
  niftk::PointSet sourcePointSet = sourceDetector.GetPoints();

  REQUIRE(sourcePointSet.size() >= 4);

  cv::Mat dummyIntrinsics;
  cv::Mat dummyDistortion;
  cv::Mat outputHomography;

  niftk::ChessboardPointDetector targetDetector(corners);
  targetDetector.SetImage(&greyImage2);
  niftk::PointSet targetPointSet = targetDetector.GetPoints();

  REQUIRE(targetPointSet.size() >= 4);
  REQUIRE(targetPointSet.size() == sourcePointSet.size());

  niftk::PointSet transformedPoints;
  niftk::WarpImageByCorrespondingPoints(greyImage1,
                                        dummyIntrinsics,
                                        dummyDistortion,
                                        sourcePointSet,
                                        targetPointSet,
                                        greyImage1.size(),
                                        outputHomography,
                                        warpedImage,
                                        transformedPoints
                                       );

  REQUIRE(greyImage2.rows == warpedImage.rows);
  REQUIRE(greyImage2.cols == warpedImage.cols);

  // Check we got points out of warped image
  niftk::ChessboardPointDetector warpedDetector(corners);
  warpedDetector.SetImage(&warpedImage);
  niftk::PointSet warpedPointSet = warpedDetector.GetPoints();
  REQUIRE(warpedPointSet.size() >= 4);
  REQUIRE(warpedPointSet.size() == sourcePointSet.size());

  // Transfer back, and check that RMS error is below a threshold.
  cv::Point2d rmsForEachAxis;
  double rms = niftk::ComputeRMSDifferenceBetweenMatchingPoints(warpedPointSet, targetPointSet, rmsForEachAxis);
  REQUIRE(fabs(rms - expectedRMS) < 0.01);

  std::cout << "rms=" << rms << ", for each axis=" << rmsForEachAxis << std::endl;
}
