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
#include <niftkIterativeStereoCameraCalibration.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkIOUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>

TEST_CASE( "Iterative Stereo Chessboard", "[StereoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  16;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkIterativeStereChessboardCameraCalibrationTest modelFileName cornersInX cornersInY referenceWidth referenceHeight fileOfPoints eRMSLeft eRMSRight eR1 eR2 eR3 eT1 eT2 eT3 image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  std::string modelFileName = niftk::argv[1];
  int numberInternalCornersInX = atoi(niftk::argv[2]);
  int numberInternalCornersInY = atoi(niftk::argv[3]);
  int widthOfReferenceImage = atoi(niftk::argv[4]);
  int heightOfReferenceImage = atoi(niftk::argv[5]);
  std::string fileOfReferencePoints = niftk::argv[6];
  int zeroDistortion = atoi(niftk::argv[7]);
  float eR1 = atof(niftk::argv[8]);
  float eR2 = atof(niftk::argv[9]);
  float eR3 = atof(niftk::argv[10]);
  float eT1 = atof(niftk::argv[11]);
  float eT2 = atof(niftk::argv[12]);
  float eT3 = atof(niftk::argv[13]);

  if (numberInternalCornersInX < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInX < 2.";
  }
  if (numberInternalCornersInY < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInY < 2.";
  }

  // Should have an even number of images left.
  if ((niftk::argc - 14) % 2 != 0)
  {
    niftkNiftyCalThrow() << "Should have an even number of images.";
  }

  // Loads "model"
  niftk::Model3D model = niftk::LoadModel3D(modelFileName);
  REQUIRE( model.size() == numberInternalCornersInY*numberInternalCornersInX );

  // Loads image data.
  cv::Size2i corners(numberInternalCornersInX, numberInternalCornersInY);
  cv::Size2i imageSize;

  // Loads all image data.
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImagesLeft;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarpingLeft;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImagesRight;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarpingRight;

  for (int i = 14; i < niftk::argc; i++)
  {
    cv::Mat image = cv::imread(niftk::argv[i]);
    imageSize.width = image.cols;
    imageSize.height = image.rows;

    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);

    std::cout << "i=" << i << ", file=" << niftk::argv[i] << std::endl;

    if (i-14 < (niftk::argc-14)/2)
    {
      std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::OpenCVChessboardPointDetector(corners));
      originalImagesLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
      dynamic_cast<niftk::OpenCVChessboardPointDetector*>(originalImagesLeft.back().first.get())->SetImage(&(originalImagesLeft.back().second));

      cv::Mat greyImageClone = greyImage.clone();
      std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::OpenCVChessboardPointDetector(corners));
      imagesForWarpingLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
      dynamic_cast<niftk::OpenCVChessboardPointDetector*>(imagesForWarpingLeft.back().first.get())->SetImage(&(imagesForWarpingLeft.back().second));

      std::cout << " left." << std::endl;
    }
    else
    {
      std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::OpenCVChessboardPointDetector(corners));
      originalImagesRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
      dynamic_cast<niftk::OpenCVChessboardPointDetector*>(originalImagesRight.back().first.get())->SetImage(&(originalImagesRight.back().second));

      cv::Mat greyImageClone = greyImage.clone();
      std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::OpenCVChessboardPointDetector(corners));
      imagesForWarpingRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
      dynamic_cast<niftk::OpenCVChessboardPointDetector*>(imagesForWarpingRight.back().first.get())->SetImage(&(imagesForWarpingRight.back().second));

      std::cout << " right." << std::endl;
    }
  }

  std::cout << "originalImagesLeft.size()=" << originalImagesLeft.size() << std::endl;
  REQUIRE( originalImagesLeft.size() >= 1 );
  std::cout << "originalImagesRight.size()=" << originalImagesRight.size() << std::endl;
  REQUIRE( originalImagesRight.size() >= 1 );
  REQUIRE( originalImagesLeft.size()  == originalImagesRight.size());

  std::pair< cv::Size2i, niftk::PointSet> referenceImageData;
  referenceImageData.first = cv::Size2i(widthOfReferenceImage, heightOfReferenceImage);
  referenceImageData.second = niftk::LoadPointSet(fileOfReferencePoints);

  int flags = 0;
  if (zeroDistortion == 1)
  {
    flags = cv::CALIB_ZERO_TANGENT_DIST
        | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2
        | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4
        | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
  }

  cv::Mat intrinsicLeft;
  cv::Mat distortionLeft;
  std::vector<cv::Mat> rvecsLeft;
  std::vector<cv::Mat> tvecsLeft;

  cv::Mat intrinsicRight;
  cv::Mat distortionRight;
  std::vector<cv::Mat> rvecsRight;
  std::vector<cv::Mat> tvecsRight;

  cv::Mat essentialMatrix;
  cv::Mat fundamentalMatrix;
  cv::Mat left2RightRotation;
  cv::Mat left2RightTranslation;

  double rms = niftk::IterativeStereoCameraCalibration(
        model,
        referenceImageData,
        originalImagesLeft,
        originalImagesRight,
        imageSize,
        imagesForWarpingLeft,
        intrinsicLeft,
        distortionLeft,
        rvecsLeft,
        tvecsLeft,
        imagesForWarpingRight,
        intrinsicRight,
        distortionRight,
        rvecsRight,
        tvecsRight,
        left2RightRotation,
        left2RightTranslation,
        essentialMatrix,
        fundamentalMatrix,
        flags
        );

  std::cout << "R1=" << left2RightRotation.at<double>(0,0) << std::endl;
  std::cout << "R2=" << left2RightRotation.at<double>(0,1) << std::endl;
  std::cout << "R3=" << left2RightRotation.at<double>(0,2) << std::endl;
  std::cout << "T1=" << left2RightTranslation.at<double>(0,0) << std::endl;
  std::cout << "T2=" << left2RightTranslation.at<double>(0,1) << std::endl;
  std::cout << "T3=" << left2RightTranslation.at<double>(0,2) << std::endl;
  std::cout << "RMS=" << rms << std::endl;

  double tolerance = 0.005;
  REQUIRE( fabs(left2RightRotation.at<double>(0,0) - eR1) < tolerance );
  REQUIRE( fabs(left2RightRotation.at<double>(0,1) - eR2) < tolerance );
  REQUIRE( fabs(left2RightRotation.at<double>(0,2) - eR3) < tolerance );
  REQUIRE( fabs(left2RightTranslation.at<double>(0,0) - eT1) < tolerance );
  REQUIRE( fabs(left2RightTranslation.at<double>(0,1) - eT2) < tolerance );
  REQUIRE( fabs(left2RightTranslation.at<double>(0,2) - eT3) < tolerance );
}
