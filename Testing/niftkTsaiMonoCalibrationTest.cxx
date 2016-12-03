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
#include <niftkCirclesPointDetector.h>
#include <niftkIOUtilities.h>
#include <niftkPointUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <fstream>

TEST_CASE( "Tsai mono", "[mono]" ) {

  int expectedNumberOfArguments =  15;
  if (niftk::argc < expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkTsaiMonoCalibrationTest image.png model.txt dotsInX dotsInY nx ny scaleX scaleY fx fy cx cy distortion rms" << std::endl;
    REQUIRE( niftk::argc >= expectedNumberOfArguments);
  }

  std::string imageFileName = niftk::argv[1];
  std::string modelFileName = niftk::argv[2];
  int dotsInX = atoi(niftk::argv[3]);
  int dotsInY = atoi(niftk::argv[4]);
  int nx = atoi(niftk::argv[5]);
  int ny = atoi(niftk::argv[6]);
  float sx = atof(niftk::argv[7]);
  float sy = atof(niftk::argv[8]);
  float eFx = atof(niftk::argv[9]);
  float eFy = atof(niftk::argv[10]);
  float eCx = atof(niftk::argv[11]);
  float eCy = atof(niftk::argv[12]);
  float dist = atof(niftk::argv[13]);
  float expectedRMS = atof(niftk::argv[14]);

  // Loads "model"
  niftk::Model3D model = niftk::LoadModel3D(modelFileName);
  REQUIRE( model.size() == dotsInX*dotsInY );

  // Loads image data.
  cv::Size2i patternSize(dotsInX, dotsInY);

  niftk::PointSet imagePoints;
  cv::Mat intrinsic;
  cv::Mat distortion;
  cv::Mat rvec;
  cv::Mat tvec;
  cv::Size imageSize;

  cv::Mat image = cv::imread(imageFileName);
  if (image.rows > 0 && image.cols > 0)
  {
    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);

    imageSize.width = greyImage.cols;
    imageSize.height = greyImage.rows;
    REQUIRE( imageSize.width == nx );
    REQUIRE( imageSize.height == ny );

    if (niftk::ModelIsPlanar(model))
    {
      // Coplanar case.
      niftk::CirclesPointDetector detector(patternSize, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING);
      detector.SetImage(&greyImage);
      detector.SetImageScaleFactor(cv::Point2d(sx, sy), false);

      imagePoints = detector.GetPoints();
      REQUIRE( imagePoints.size() == dotsInX*dotsInY );
    }
    else
    {
      // Noncoplanar case.
    }
  }

  double sensorScaleInX = 1;

  cv::Point2d sensorDimensions;
  sensorDimensions.x = 1;
  sensorDimensions.y = 1;

  cv::Size scaledSize(imageSize.width * sx, imageSize.height * sy);
  double rms = niftk::TsaiMonoCameraCalibration(model, imagePoints, scaledSize, sensorDimensions, nx, sensorScaleInX, intrinsic, distortion, rvec, tvec, true);

  std::cout << "RMS=" << rms << std::endl;
  std::cout << "Fx=" << intrinsic.at<double>(0,0) << std::endl;
  std::cout << "Fy=" << intrinsic.at<double>(1,1) << std::endl;
  std::cout << "Sx=" << sensorScaleInX << std::endl;
  std::cout << "Cx=" << intrinsic.at<double>(0,2) << std::endl;
  std::cout << "Cy=" << intrinsic.at<double>(1,2) << std::endl;
  std::cout << "d1=" << distortion.at<double>(0,0) << std::endl;
  std::cout << "d2=" << distortion.at<double>(0,1) << std::endl;
  std::cout << "d3=" << distortion.at<double>(0,2) << std::endl;
  std::cout << "d4=" << distortion.at<double>(0,3) << std::endl;
  std::cout << "R1=" << rvec.at<double>(0,0) << std::endl;
  std::cout << "R2=" << rvec.at<double>(0,1) << std::endl;
  std::cout << "R3=" << rvec.at<double>(0,2) << std::endl;
  std::cout << "Tx=" << tvec.at<double>(0,0) << std::endl;
  std::cout << "Ty=" << tvec.at<double>(0,1) << std::endl;
  std::cout << "Tz=" << tvec.at<double>(0,2) << std::endl;

  double tol = 0.01;

  REQUIRE( fabs(intrinsic.at<double>(0,0) - eFx) < tol );
  REQUIRE( fabs(intrinsic.at<double>(1,1) - eFy) < tol );
  REQUIRE( fabs(intrinsic.at<double>(0,2) - eCx) < tol );
  REQUIRE( fabs(intrinsic.at<double>(1,2) - eCy) < tol );
  REQUIRE( fabs(distortion.at<double>(0,0) - dist) < tol );
  REQUIRE( fabs(distortion.at<double>(0,1) - 0) < tol );
  REQUIRE( fabs(distortion.at<double>(0,2) - 0) < tol );
  REQUIRE( fabs(distortion.at<double>(0,3) - 0) < tol );
  REQUIRE( fabs(rms - expectedRMS) < tol );
}
