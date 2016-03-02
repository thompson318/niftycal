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
#include <niftkIPoint2DDetector.h>
#include <niftkOpenCVChessboardPointDetector.h>
#include <niftkIterativeMonoCameraCalibration.h>
#include <niftkIOUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>

TEST_CASE( "Iterative Mono Chessboard", "[MonoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  15;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkIterativeMonoChessboardCameraCalibrationTest squareSizeInMillimetres cornersInX cornersInY eRMS eFx eFy eCx eCy eK1 eK2 eP1 eP2 image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  float squareSizeInMillimetres = atof(niftk::argv[1]);
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

  if (numberInternalCornersInX < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInX < 2";
  }
  if (numberInternalCornersInY < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInY < 2";
  }

  // Generates "model", as we know its a chessboard!
  niftk::Model3D model;
  int counter = 0;
  for (int y = 0; y < numberInternalCornersInY; y++)
  {
    for (int x = 0; x < numberInternalCornersInX; x++)
    {
      niftk::Point3D tmp;
      tmp.point.x = x*squareSizeInMillimetres;
      tmp.point.y = y*squareSizeInMillimetres;
      tmp.point.z = 0;
      tmp.id = counter;

      model.insert(niftk::IdPoint3D(tmp.id, tmp));
      counter++;
    }
  }
  REQUIRE( model.size() == numberInternalCornersInY*numberInternalCornersInX );
  cv::Size2i corners(numberInternalCornersInX, numberInternalCornersInY);
  cv::Size2i imageSize;

  // Loads all image data.
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImages;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarping;
  for (int i = 13; i < niftk::argc; i++)
  {
    cv::Mat image = cv::imread(niftk::argv[i]);
    imageSize.width = image.cols;
    imageSize.height = image.rows;

    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);

    std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::OpenCVChessboardPointDetector(greyImage, corners));
    originalImages.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));

    cv::Mat greyImageClone = greyImage.clone();
    std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::OpenCVChessboardPointDetector(greyImageClone, corners));
    imagesForWarping.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
  }

  REQUIRE(originalImages.size() == niftk::argc-13);
  REQUIRE(imagesForWarping.size() == niftk::argc-13);

  cv::Mat intrinsic;
  cv::Mat distortion;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  double rms = niftk::IterativeMonoCameraCalibration(
        model,
        originalImages,
        imagesForWarping,
        imageSize,
        intrinsic,
        distortion,
        rvecs,
        tvecs
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

  double tolerance = 0.005;
  REQUIRE( fabs(rms - eRMS) < 0.001 );
  REQUIRE( fabs(intrinsic.at<double>(0,0) - eFx) < tolerance );
  REQUIRE( fabs(intrinsic.at<double>(1,1) - eFy) < tolerance );
  REQUIRE( fabs(intrinsic.at<double>(0,2) - eCx) < tolerance );
  REQUIRE( fabs(intrinsic.at<double>(1,2) - eCy) < tolerance );
  REQUIRE( fabs(distortion.at<double>(0,0) - eK1) < tolerance );
  REQUIRE( fabs(distortion.at<double>(0,1) - eK2) < tolerance );
  REQUIRE( fabs(distortion.at<double>(0,2) - eP1) < tolerance );
  REQUIRE( fabs(distortion.at<double>(0,3) - eP2) < tolerance );

}