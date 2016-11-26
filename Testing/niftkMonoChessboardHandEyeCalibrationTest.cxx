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
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkChessboardPointDetector.h>
#include <niftkZhangCameraCalibration.h>
#include <niftkHandEyeCalibration.h>
#include <niftkIOUtilities.h>
#include <niftkMatrixUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>
#include <chrono>

TEST_CASE( "Mono HandEye", "[MonoCalibration]" ) {

  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();

  int expectedMinimumNumberOfArguments =  11;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkMonoChessboardHandEyeCalibrationTest modelFileName cornersInX cornersInY expectedHandEye.4x4 image1.png image2.png ... imageN.png tracking1.4x4 tracking2.4x4 ... trackingN.4x4 " << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  if ((niftk::argc - 5)%2 != 0)
  {
    niftkNiftyCalThrow() << "After first 5 arguments, should have equal number of images and tracker matrices." << std::endl;
  }

  std::string modelFileName = niftk::argv[1];
  int numberInternalCornersInX = atoi(niftk::argv[2]);
  int numberInternalCornersInY = atoi(niftk::argv[3]);
  std::string expectedEyeHandFileName = niftk::argv[4]; // NifTK actually does Eye-Hand

  if (numberInternalCornersInX < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInX < 2";
  }
  if (numberInternalCornersInY < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInY < 2";
  }

  // Loads "model"
  niftk::Model3D model = niftk::LoadModel3D(modelFileName);
  REQUIRE( model.size() == numberInternalCornersInY*numberInternalCornersInX );

  // Loads "expected" result.
  cv::Mat expectedMatrix = niftk::LoadMatrix(expectedEyeHandFileName);
  cv::Matx44d expectedEyeHand = expectedMatrix;

  // Loads image data.
  cv::Size2i corners(numberInternalCornersInX, numberInternalCornersInY);
  cv::Size2i imageSize;
  cv::Point2d scaleFactors;
  scaleFactors.x = 1;
  scaleFactors.y = 2;
  niftk::PointSet pointSet;
  std::list<niftk::PointSet> listOfPoints;

  int snapshots = (niftk::argc -5)/2;

  std::cout << "Loaded model and expected result." << std::endl;
  std::cout << "argc=" << niftk::argc << ", snapshots=" << snapshots << std::endl;

  for (int i = 5; i < snapshots+5; i++)
  {
    std::cout << "Loading i=" << i << ", file=" << niftk::argv[i] << std::endl;

    cv::Mat image = cv::imread(niftk::argv[i]);
    if (image.rows > 0 && image.cols > 0)
    {
      cv::Mat greyImage;
      cv::cvtColor(image, greyImage, CV_BGR2GRAY);

      imageSize.width = greyImage.cols;
      imageSize.height = greyImage.rows;

      niftk::ChessboardPointDetector detector(corners);
      detector.SetImage(&greyImage);
      detector.SetImageScaleFactor(scaleFactors);
      pointSet = detector.GetPoints();

      if (pointSet.size() > 0)
      {
        listOfPoints.push_back(pointSet);
      }
      std::cout << "Loaded image in argument=" << i << ", total number of images=" << listOfPoints.size() << std::endl;
    }
  }

  REQUIRE( listOfPoints.size() >= 2 );

  cv::Mat intrinsic;
  cv::Mat distortion;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  std::chrono::time_point<std::chrono::system_clock> endStartup = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = endStartup - start;
  std::cout << "TIME:Startup=" << elapsed_seconds.count() << std::endl;

  double rms = niftk::ZhangMonoCameraCalibration(model,
                                                 listOfPoints,
                                                 imageSize,
                                                 intrinsic,
                                                 distortion,
                                                 rvecs,
                                                 tvecs
                                                 );

  std::cout << "Mono intrinsic calibration RMS=" << rms << std::endl;

  std::chrono::time_point<std::chrono::system_clock> endCalib = std::chrono::system_clock::now();
  elapsed_seconds = endCalib - endStartup;
  std::cout << "TIME:Calib=" << elapsed_seconds.count() << std::endl;

  std::list<cv::Matx44d> trackingMatrices;

  for (int i = snapshots+5; i < niftk::argc; i++)
  {
    cv::Mat tmp = niftk::LoadMatrix(niftk::argv[i]);
    cv::Matx44d tmp4x4 = tmp;
    trackingMatrices.push_back(tmp4x4);
  }

  REQUIRE(trackingMatrices.size() == rvecs.size());
  REQUIRE(trackingMatrices.size() == tvecs.size());

  std::list<cv::Matx44d> cameraMatrices;
  for (int i = 0; i < rvecs.size(); i++)
  {
    cameraMatrices.push_back(niftk::RodriguesToMatrix(rvecs[i], tvecs[i]));
  }

  REQUIRE(trackingMatrices.size() == cameraMatrices.size());

  cv::Matx21d residuals;

  std::cout << "Camera matrices=" << cameraMatrices.size() << ", tracking matrices=" << trackingMatrices.size() << std::endl;

  cv::Matx44d handEye = niftk::CalculateHandEyeUsingTsaisMethod(
        trackingMatrices,
        cameraMatrices,
        residuals
        );

  std::cout << "Done hand-eye, rotation residual=" << residuals(0, 0) << ", translation residual=" << residuals(1, 0) << std::endl;

  std::chrono::time_point<std::chrono::system_clock> endTsai= std::chrono::system_clock::now();
  elapsed_seconds = endTsai - endCalib;
  std::cout << "TIME:Tsai=" << elapsed_seconds.count() << std::endl;

  cv::Matx44d eyeHand = handEye.inv(cv::DECOMP_SVD);

  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 4; c++)
    {
      std::cout << "Expected (" << r << ", " << c << ")="
                << expectedEyeHand(r, c) << ", actual="
                << eyeHand(r, c) << std::endl;
      REQUIRE(fabs(expectedEyeHand(r, c) - eyeHand(r, c)) < 0.001);
    }
  }

  double reprojectionRMS = 0;

#ifdef NIFTYCAL_WITH_ITK

  // Here, Im just testing that the stereo code runs to completion, as we only have mono data.
  cv::Matx44d stereoExtrinsics = cv::Matx44d::eye();
  handEye = niftk::CalculateHandEyeInStereoByOptimisingAllExtrinsic(model, listOfPoints, intrinsic, distortion, listOfPoints, intrinsic, distortion, trackingMatrices, cameraMatrices, stereoExtrinsics, reprojectionRMS);

  std::chrono::time_point<std::chrono::system_clock> endStereo= std::chrono::system_clock::now();
  elapsed_seconds = endStereo - endTsai;
  std::cout << "TIME:STEREO=" << elapsed_seconds.count() << std::endl;

  handEye = niftk::CalculateHandEyeByOptimisingAllExtrinsic(model, listOfPoints, trackingMatrices, cameraMatrices, intrinsic, distortion, reprojectionRMS);

  std::chrono::time_point<std::chrono::system_clock> endNDOF= std::chrono::system_clock::now();
  elapsed_seconds = endNDOF - endStereo;
  std::cout << "TIME:NDOF=" << elapsed_seconds.count() << std::endl;

  handEye = niftk::CalculateHandEyeUsingMaltisMethod(model, listOfPoints, trackingMatrices, cameraMatrices, intrinsic, distortion, reprojectionRMS);

  std::chrono::time_point<std::chrono::system_clock> endMalti= std::chrono::system_clock::now();
  elapsed_seconds = endMalti - endNDOF;
  std::cout << "TIME:Malti=" << elapsed_seconds.count() << std::endl;

#endif
}
