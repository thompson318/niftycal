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

TEST_CASE( "Tsai stereo", "[stereo]" ) {

  int expectedNumberOfArguments =  18;
  if (niftk::argc < expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkTsaiStereoCalibrationTest imageL.png imageR.png model.txt dotsInX dotsInY nx ny scaleX scaleY asymmetric fx fy cx cy distortion expectedRMS optimise3D" << std::endl;
    REQUIRE( niftk::argc >= expectedNumberOfArguments);
  }

  std::string leftImageFileName = niftk::argv[1];
  std::string rightImageFileName = niftk::argv[2];
  std::string modelFileName = niftk::argv[3];
  int dotsInX = atoi(niftk::argv[4]);
  int dotsInY = atoi(niftk::argv[5]);
  int nx = atoi(niftk::argv[6]);
  int ny = atoi(niftk::argv[7]);
  float sx = atof(niftk::argv[8]);
  float sy = atof(niftk::argv[9]);
  int asymmetric = atoi(niftk::argv[10]);
  float eFx = atof(niftk::argv[11]);
  float eFy = atof(niftk::argv[12]);
  float eCx = atof(niftk::argv[13]);
  float eCy = atof(niftk::argv[14]);
  float dist = atof(niftk::argv[15]);
  float expectedRMS = atof(niftk::argv[16]);
  int optimise3D = atoi(niftk::argv[17]);

  int flags = cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING;
  if (asymmetric == 1)
  {
    flags = cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING;
  }

  // Loads "model"
  niftk::Model3D model = niftk::LoadModel3D(modelFileName);
  REQUIRE( model.size() == dotsInX*dotsInY );

  // Loads image data.
  cv::Size2i patternSize(dotsInX, dotsInY);

  niftk::PointSet leftImagePoints;
  cv::Mat intrinsicLeft;
  cv::Mat distortionLeft;
  cv::Mat rvecLeft;
  cv::Mat tvecLeft;
  niftk::PointSet rightImagePoints;
  cv::Mat intrinsicRight;
  cv::Mat distortionRight;
  cv::Mat rvecRight;
  cv::Mat tvecRight;

  cv::Mat leftToRightRotationMatrix;
  cv::Mat leftToRightTranslationVector;
  cv::Mat essentialMatrix;
  cv::Mat fundamentalMatrix;

  cv::Size imageSize;

  cv::Mat leftImage = cv::imread(leftImageFileName);
  if (leftImage.rows > 0 && leftImage.cols > 0)
  {
    cv::Mat rightImage = cv::imread(rightImageFileName);
    if (rightImage.rows > 0 && rightImage.cols > 0)
    {
      cv::Mat leftGreyImage;
      cv::cvtColor(leftImage, leftGreyImage, CV_BGR2GRAY);

      cv::Mat rightGreyImage;
      cv::cvtColor(rightImage, rightGreyImage, CV_BGR2GRAY);

      imageSize.width = leftGreyImage.cols;
      imageSize.height = leftGreyImage.rows;
      REQUIRE( imageSize.width == nx );
      REQUIRE( imageSize.height == ny );

      if (niftk::ModelIsPlanar(model))
      {
        // Coplanar case.
        niftk::CirclesPointDetector detector(patternSize, flags);
        detector.SetImageScaleFactor(cv::Point2d(sx, sy), false);

        detector.SetImage(&leftGreyImage);
        leftImagePoints = detector.GetPoints();

        detector.SetImage(&rightGreyImage);
        rightImagePoints = detector.GetPoints();

        REQUIRE( leftImagePoints.size() == dotsInX*dotsInY );
        REQUIRE( rightImagePoints.size() == dotsInX*dotsInY );
      }
      else
      {
        // Noncoplanar case.
      }
    }
  }

  cv::Point2d sensorDimensions;
  sensorDimensions.x = 1;
  sensorDimensions.y = 1;

  cv::Size scaledSize(imageSize.width * sx, imageSize.height * sy);

  double rmsLeft = niftk::TsaiMonoCameraCalibration(model,
                                                    leftImagePoints,
                                                    scaledSize,
                                                    sensorDimensions,
                                                    intrinsicLeft,
                                                    distortionLeft,
                                                    rvecLeft,
                                                    tvecLeft,
                                                    true);

  double rmsRight = niftk::TsaiMonoCameraCalibration(model,
                                                     rightImagePoints,
                                                     scaledSize,
                                                     sensorDimensions,
                                                     intrinsicRight,
                                                     distortionRight,
                                                     rvecRight,
                                                     tvecRight,
                                                     true);

  cv::Matx21d rms = niftk::TsaiStereoCameraCalibration(model, leftImagePoints, rightImagePoints,
                                                  scaledSize,
                                                  intrinsicLeft,
                                                  distortionLeft,
                                                  rvecLeft,
                                                  tvecLeft,
                                                  intrinsicRight,
                                                  distortionRight,
                                                  rvecRight,
                                                  tvecRight,
                                                  leftToRightRotationMatrix,
                                                  leftToRightTranslationVector,
                                                  essentialMatrix,
                                                  fundamentalMatrix,
                                                  CV_CALIB_USE_INTRINSIC_GUESS
                                                  | CV_CALIB_FIX_INTRINSIC,
                                                  optimise3D
                                                  );
  std::cout << "Fx=" << intrinsicLeft.at<double>(0,0) << std::endl;
  std::cout << "Fy=" << intrinsicLeft.at<double>(1,1) << std::endl;
  std::cout << "Cx=" << intrinsicLeft.at<double>(0,2) << std::endl;
  std::cout << "Cy=" << intrinsicLeft.at<double>(1,2) << std::endl;
  std::cout << "d1=" << distortionLeft.at<double>(0,0) << std::endl;
  std::cout << "d2=" << distortionLeft.at<double>(0,1) << std::endl;
  std::cout << "d3=" << distortionLeft.at<double>(0,2) << std::endl;
  std::cout << "d4=" << distortionLeft.at<double>(0,3) << std::endl;
  std::cout << "R1=" << rvecLeft.at<double>(0,0) << std::endl;
  std::cout << "R2=" << rvecLeft.at<double>(0,1) << std::endl;
  std::cout << "R3=" << rvecLeft.at<double>(0,2) << std::endl;
  std::cout << "Tx=" << tvecLeft.at<double>(0,0) << std::endl;
  std::cout << "Ty=" << tvecLeft.at<double>(0,1) << std::endl;
  std::cout << "Tz=" << tvecLeft.at<double>(0,2) << std::endl;
  std::cout << "Fx=" << intrinsicRight.at<double>(0,0) << std::endl;
  std::cout << "Fy=" << intrinsicRight.at<double>(1,1) << std::endl;
  std::cout << "Cx=" << intrinsicRight.at<double>(0,2) << std::endl;
  std::cout << "Cy=" << intrinsicRight.at<double>(1,2) << std::endl;
  std::cout << "d1=" << distortionRight.at<double>(0,0) << std::endl;
  std::cout << "d2=" << distortionRight.at<double>(0,1) << std::endl;
  std::cout << "d3=" << distortionRight.at<double>(0,2) << std::endl;
  std::cout << "d4=" << distortionRight.at<double>(0,3) << std::endl;
  std::cout << "R1=" << rvecRight.at<double>(0,0) << std::endl;
  std::cout << "R2=" << rvecRight.at<double>(0,1) << std::endl;
  std::cout << "R3=" << rvecRight.at<double>(0,2) << std::endl;
  std::cout << "Tx=" << tvecRight.at<double>(0,0) << std::endl;
  std::cout << "Ty=" << tvecRight.at<double>(0,1) << std::endl;
  std::cout << "Tz=" << tvecRight.at<double>(0,2) << std::endl;
  std::cout << "RMS left=" << rmsLeft << std::endl;
  std::cout << "RMS right=" << rmsRight << std::endl;
  std::cout << "RMS 2D=" << rms(0, 0) << std::endl;
  std::cout << "RMS 3D=" << rms(1, 0) << std::endl;

  double tol = 0.01;

  REQUIRE( fabs(intrinsicLeft.at<double>(0,0) - eFx) < tol );
  REQUIRE( fabs(intrinsicLeft.at<double>(1,1) - eFy) < tol );
  REQUIRE( fabs(intrinsicLeft.at<double>(0,2) - eCx) < tol );
  REQUIRE( fabs(intrinsicLeft.at<double>(1,2) - eCy) < tol );
  REQUIRE( fabs(distortionLeft.at<double>(0,0) - dist) < tol );
  REQUIRE( fabs(distortionLeft.at<double>(0,1) - 0) < tol );
  REQUIRE( fabs(distortionLeft.at<double>(0,2) - 0) < tol );
  REQUIRE( fabs(distortionLeft.at<double>(0,3) - 0) < tol );
  REQUIRE( fabs(rms(1, 0) - expectedRMS) < tol );
}
