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
#include <niftkChessboardPointDetector.h>
#include <niftkIterativeStereoCameraCalibration.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkIOUtilities.h>
#include <niftkPointUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>

TEST_CASE( "Iterative Stereo Chessboard", "[StereoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  15;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkIterativeStereChessboardCameraCalibrationTest modelImage modelFileName cornersInX cornersInY fileOfPoints eRMSLeft eRMSRight eR1 eR2 eR3 eT1 eT2 eT3 image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  std::string modelImage = niftk::argv[1];
  std::string modelFileName = niftk::argv[2];
  int numberInternalCornersInX = atoi(niftk::argv[3]);
  int numberInternalCornersInY = atoi(niftk::argv[4]);
  std::string fileOfReferencePoints = niftk::argv[5];
  int zeroDistortion = atoi(niftk::argv[6]);
  float eR1 = atof(niftk::argv[7]);
  float eR2 = atof(niftk::argv[8]);
  float eR3 = atof(niftk::argv[9]);
  float eT1 = atof(niftk::argv[10]);
  float eT2 = atof(niftk::argv[11]);
  float eT3 = atof(niftk::argv[12]);

  if (numberInternalCornersInX < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInX < 2.";
  }
  if (numberInternalCornersInY < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInY < 2.";
  }

  // Should have an even number of images left.
  if ((niftk::argc - 13) % 2 != 0)
  {
    niftkNiftyCalThrow() << "Should have an even number of images.";
  }

  cv::Mat referenceImage = cv::imread(modelImage);
  cv::Mat referenceImageGreyScale;
  cv::cvtColor(referenceImage, referenceImageGreyScale, CV_BGR2GRAY);

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

  for (int i = 13; i < niftk::argc; i++)
  {
    cv::Mat image = cv::imread(niftk::argv[i]);
    imageSize.width = image.cols;
    imageSize.height = image.rows;

    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);

    std::cout << "i=" << i << ", file=" << niftk::argv[i] << std::endl;

    if (i-13 < (niftk::argc-13)/2)
    {
      std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::ChessboardPointDetector(corners));
      originalImagesLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
      dynamic_cast<niftk::ChessboardPointDetector*>(originalImagesLeft.back().first.get())->SetImage(&(originalImagesLeft.back().second));

      cv::Mat greyImageClone = greyImage.clone();
      std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::ChessboardPointDetector(corners));
      imagesForWarpingLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
      dynamic_cast<niftk::ChessboardPointDetector*>(imagesForWarpingLeft.back().first.get())->SetImage(&(imagesForWarpingLeft.back().second));

      std::cout << " left." << std::endl;
    }
    else
    {
      std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::ChessboardPointDetector(corners));
      originalImagesRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
      dynamic_cast<niftk::ChessboardPointDetector*>(originalImagesRight.back().first.get())->SetImage(&(originalImagesRight.back().second));

      cv::Mat greyImageClone = greyImage.clone();
      std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::ChessboardPointDetector(corners));
      imagesForWarpingRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
      dynamic_cast<niftk::ChessboardPointDetector*>(imagesForWarpingRight.back().first.get())->SetImage(&(imagesForWarpingRight.back().second));

      std::cout << " right." << std::endl;
    }
  }

  std::cout << "originalImagesLeft.size()=" << originalImagesLeft.size() << std::endl;
  REQUIRE( originalImagesLeft.size() >= 1 );
  std::cout << "originalImagesRight.size()=" << originalImagesRight.size() << std::endl;
  REQUIRE( originalImagesRight.size() >= 1 );
  REQUIRE( originalImagesLeft.size()  == originalImagesRight.size());

  std::pair< cv::Mat, niftk::PointSet> referenceImageData;
  referenceImageData.first = referenceImageGreyScale;
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
  cv::Mat rightToLeftRotationMatrix;
  cv::Mat rightToleftTranslationVector;

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
        rightToLeftRotationMatrix,
        rightToleftTranslationVector,
        essentialMatrix,
        fundamentalMatrix,
        flags
        );

  double tolerance = 0.005;

  cv::Mat rvec;
  cv::Rodrigues(rightToLeftRotationMatrix, rvec);

  std::cout << "R1=" << rvec.at<double>(0,0) << std::endl;
  std::cout << "R2=" << rvec.at<double>(0,1) << std::endl;
  std::cout << "R3=" << rvec.at<double>(0,2) << std::endl;
  std::cout << "T1=" << rightToleftTranslationVector.at<double>(0,0) << std::endl;
  std::cout << "T2=" << rightToleftTranslationVector.at<double>(1,0) << std::endl;
  std::cout << "T3=" << rightToleftTranslationVector.at<double>(2,0) << std::endl;
  std::cout << "RMS=" << rms << std::endl;

  REQUIRE( fabs(rvec.at<double>(0,0) - eR1) < tolerance );
  REQUIRE( fabs(rvec.at<double>(0,1) - eR2) < tolerance );
  REQUIRE( fabs(rvec.at<double>(0,2) - eR3) < tolerance );
  REQUIRE( fabs(rightToleftTranslationVector.at<double>(0,0) - eT1) < tolerance );
  REQUIRE( fabs(rightToleftTranslationVector.at<double>(1,0) - eT2) < tolerance );
  REQUIRE( fabs(rightToleftTranslationVector.at<double>(2,0) - eT3) < tolerance );

/*
  cv::Mat imageLeft = cv::imread(niftk::argv[13]);
  cv::Mat greyImageLeft;
  cv::cvtColor(imageLeft, greyImageLeft, CV_BGR2GRAY);
  niftk::ChessboardPointDetector detectorLeft(corners);
  detectorLeft.SetImage(&greyImageLeft);
  niftk::PointSet pointSetLeft = detectorLeft.GetPoints();

  cv::Mat imageRight = cv::imread(niftk::argv[21]);
  cv::Mat greyImageRight;
  cv::cvtColor(imageRight, greyImageRight, CV_BGR2GRAY);

  cv::Mat rightImageWithLines = niftk::DrawEpiLines(pointSetLeft,
                                                    intrinsicLeft,
                                                    distortionLeft,
                                                    1,
                                                    fundamentalMatrix,
                                                    greyImageRight,
                                                    intrinsicRight,
                                                    distortionRight
                                                   );
  cv::imwrite("/tmp/matt.epi.png", rightImageWithLines);
*/
}
