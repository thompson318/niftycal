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
#include <niftkIOUtilities.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>

TEST_CASE( "Mono Chessboard", "[MonoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  19;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkMonoChessboardCameraCalibrationTest modelFileName cornersInX cornersInY eRMS eFx eFy eCx eCy eK1 eK2 eP1 eP2 zeroDist fxTolerance cxTolerance distortionTolerance image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  std::string modelFileName = niftk::argv[1];
  int numberInternalCornersInX = atoi(niftk::argv[2]);
  int numberInternalCornersInY = atoi(niftk::argv[3]);
  float eRMS = atof(niftk::argv[4]);
  float eFx = atof(niftk::argv[5]);
  float eFy = atof(niftk::argv[6]);
  float eCx = atof(niftk::argv[7]);
  float eCy = atof(niftk::argv[8]);
  float eK1 = atof(niftk::argv[9]);
  float eK2 = atof(niftk::argv[10]);
  float eP1 = atof(niftk::argv[11]);
  float eP2 = atof(niftk::argv[12]);
  int zeroDistortion = atoi(niftk::argv[13]);
  float fxTol = atof(niftk::argv[14]);
  float cxTol = atof(niftk::argv[15]);
  float distTol = atof(niftk::argv[16]);

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

  // Loads image data.
  cv::Size2i corners(numberInternalCornersInX, numberInternalCornersInY);

  niftk::PointSet pointSet;
  std::list<niftk::PointSet> listOfPoints;
  cv::Size2i imageSize;

  for (int i = 17; i < niftk::argc; i++)
  {
    cv::Mat image = cv::imread(niftk::argv[i]);
    if (image.rows > 0 && image.cols > 0)
    {
      cv::Mat greyImage;
      cv::cvtColor(image, greyImage, CV_BGR2GRAY);

      imageSize.width = greyImage.cols;
      imageSize.height = greyImage.rows;

      niftk::ChessboardPointDetector detector(corners);
      detector.SetImage(&greyImage);
      pointSet = detector.GetPoints();

      if (pointSet.size() > 0)
      {
        listOfPoints.push_back(pointSet);
      }
    }
  }

  REQUIRE( listOfPoints.size() >= 2 );

  int flags = 0;
  if (zeroDistortion == 1)
  {
    flags = cv::CALIB_ZERO_TANGENT_DIST
        | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2
        | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4
        | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
  }

  cv::Mat intrinsic;
  cv::Mat distortion;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  double rms = niftk::ZhangMonoCameraCalibration(model,
                                                 listOfPoints,
                                                 imageSize,
                                                 intrinsic,
                                                 distortion,
                                                 rvecs,
                                                 tvecs,
                                                 flags
                                                 );

  std::cout << "RMS=" << rms << std::endl;
  std::cout << "Fx=" << intrinsic.at<double>(0,0) << std::endl;
  std::cout << "Fy=" << intrinsic.at<double>(1,1) << std::endl;
  std::cout << "Cx=" << intrinsic.at<double>(0,2) << std::endl;
  std::cout << "Cy=" << intrinsic.at<double>(1,2) << std::endl;
  std::cout << "K1=" << distortion.at<double>(0,0) << std::endl;
  std::cout << "K2=" << distortion.at<double>(0,1) << std::endl;
  std::cout << "P1=" << distortion.at<double>(0,2) << std::endl;
  std::cout << "P2=" << distortion.at<double>(0,3) << std::endl;

  REQUIRE( fabs(rms - eRMS) < 0.001 );
  REQUIRE( fabs(intrinsic.at<double>(0,0) - eFx) < fxTol );
  REQUIRE( fabs(intrinsic.at<double>(1,1) - eFy) < fxTol );
  REQUIRE( fabs(intrinsic.at<double>(0,2) - eCx) < cxTol );
  REQUIRE( fabs(intrinsic.at<double>(1,2) - eCy) < cxTol );
  REQUIRE( fabs(distortion.at<double>(0,0) - eK1) < distTol );
  REQUIRE( fabs(distortion.at<double>(0,1) - eK2) < distTol );
  REQUIRE( fabs(distortion.at<double>(0,2) - eP1) < distTol );
  REQUIRE( fabs(distortion.at<double>(0,3) - eP2) < distTol );
}
