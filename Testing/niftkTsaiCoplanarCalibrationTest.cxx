/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "catch.hpp"
#include <niftkCatchMain.h>
#include <niftkNiftyCalTypes.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkTsaiCameraCalibration.h>
#include <niftkChessboardPointDetector.h>
#include <niftkIOUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <fstream>

TEST_CASE( "Tsai mono", "[mono]" ) {

  int expectedNumberOfArguments =  12;
  if (niftk::argc < expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkTsaiCoplanarCalibrationTest image.png model.txt cornersX cornersY nx ny fx fy cx cy distortion" << std::endl;
    REQUIRE( niftk::argc >= expectedNumberOfArguments);
  }

  std::string imageFileName = niftk::argv[1];
  std::string modelFileName = niftk::argv[2];
  int numberInternalCornersInX = atoi(niftk::argv[3]);
  int numberInternalCornersInY = atoi(niftk::argv[4]);
  int nx = atoi(niftk::argv[5]);
  int ny = atoi(niftk::argv[6]);
  float eFx = atof(niftk::argv[7]);
  float eFy = atof(niftk::argv[8]);
  float eCx = atof(niftk::argv[9]);
  float eCy = atof(niftk::argv[10]);
  float dist = atof(niftk::argv[11]);

  // Loads "model"
  niftk::Model3D model = niftk::LoadModel3D(modelFileName);
  REQUIRE( model.size() == numberInternalCornersInY*numberInternalCornersInX );

  // Loads image data.
  cv::Size2i corners(numberInternalCornersInX, numberInternalCornersInY);

  niftk::PointSet imagePoints;
  cv::Mat intrinsic;
  cv::Mat distortion;
  cv::Mat rvec;
  cv::Mat tvec;
  cv::Size imageSize;
  cv::Point2d sensorDimensions;
  sensorDimensions.x = 1;
  sensorDimensions.y = 1;

  cv::Mat image = cv::imread(imageFileName);
  if (image.rows > 0 && image.cols > 0)
  {
    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);

    imageSize.width = greyImage.cols;
    imageSize.height = greyImage.rows;
    REQUIRE( imageSize.width == nx );
    REQUIRE( imageSize.height == ny );

    niftk::ChessboardPointDetector detector(corners);
    detector.SetImage(&greyImage);
    imagePoints = detector.GetPoints();
    REQUIRE( imagePoints.size() == numberInternalCornersInX*numberInternalCornersInY );
  }

  double rms = niftk::TsaiMonoCoplanarCameraCalibration(model, imagePoints, imageSize, sensorDimensions, nx, intrinsic, distortion, rvec, tvec);

  std::cout << "RMS=" << rms << std::endl;
  std::cout << "Fx=" << intrinsic.at<double>(0,0) << std::endl;
  std::cout << "Fy=" << intrinsic.at<double>(1,1) << std::endl;
  std::cout << "Cx=" << intrinsic.at<double>(0,2) << std::endl;
  std::cout << "Cy=" << intrinsic.at<double>(1,2) << std::endl;
  std::cout << "d1=" << distortion.at<double>(0,0) << std::endl;
  std::cout << "d2=" << distortion.at<double>(0,1) << std::endl;
  std::cout << "d3=" << distortion.at<double>(0,2) << std::endl;
  std::cout << "d4=" << distortion.at<double>(0,3) << std::endl;

  double tol = 0.01;

  REQUIRE( fabs(intrinsic.at<double>(0,0) - eFx) < tol );
  REQUIRE( fabs(intrinsic.at<double>(1,1) - eFy) < tol );
  REQUIRE( fabs(intrinsic.at<double>(0,2) - eCx) < tol );
  REQUIRE( fabs(intrinsic.at<double>(1,2) - eCy) < tol );
  REQUIRE( fabs(distortion.at<double>(0,0) - dist) < tol );
  REQUIRE( fabs(distortion.at<double>(0,1) - 0) < tol );
  REQUIRE( fabs(distortion.at<double>(0,2) - 0) < tol );
  REQUIRE( fabs(distortion.at<double>(0,3) - 0) < tol );

}
