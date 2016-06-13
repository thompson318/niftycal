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
#include <niftkCirclesPointDetector.h>
#include <niftkIterativeStereoCameraCalibration.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkIOUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>

TEST_CASE( "Iterative Stereo Circles", "[StereoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  15;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkIterativeStereCirclesCameraCalibrationTest modelImage modelFileName expectedColumns expectedCirclesPerColumn fileOfPoints eRMSLeft eRMSRight eR1 eR2 eR3 eT1 eT2 eT3 image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  std::string modelImage = niftk::argv[1];
  std::string modelFileName = niftk::argv[2];
  int expectedColumns = atoi(niftk::argv[3]);
  int expectedCirclesPerColumn = atoi(niftk::argv[4]);
  std::string fileOfReferencePoints = niftk::argv[5];
  int zeroDistortion = atoi(niftk::argv[6]);
  float eR1 = atof(niftk::argv[7]);
  float eR2 = atof(niftk::argv[8]);
  float eR3 = atof(niftk::argv[9]);
  float eT1 = atof(niftk::argv[10]);
  float eT2 = atof(niftk::argv[11]);
  float eT3 = atof(niftk::argv[12]);

  if (expectedColumns < 2)
  {
    niftkNiftyCalThrow() << "expectedColumns < 2.";
  }
  if (expectedCirclesPerColumn < 2)
  {
    niftkNiftyCalThrow() << "expectedCirclesPerColumn < 2.";
  }

  // Should have an even number of images left.
  if ((niftk::argc - 13) % 2 != 0)
  {
    niftkNiftyCalThrow() << "Should have an even number of images.";
  }

  cv::Mat referenceImage = cv::imread(modelImage);

  // Loads "model"
  niftk::Model3D model = niftk::LoadModel3D(modelFileName);
  REQUIRE( model.size() == expectedColumns*expectedCirclesPerColumn );

  // Loads image data.
  cv::Size2i patternSize(expectedCirclesPerColumn, expectedColumns);
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
      std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::CirclesPointDetector(patternSize));
      originalImagesLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
      dynamic_cast<niftk::CirclesPointDetector*>(originalImagesLeft.back().first.get())->SetImage(&(originalImagesLeft.back().second));

      cv::Mat greyImageClone = greyImage.clone();
      std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::CirclesPointDetector(patternSize));
      imagesForWarpingLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
      dynamic_cast<niftk::CirclesPointDetector*>(imagesForWarpingLeft.back().first.get())->SetImage(&(imagesForWarpingLeft.back().second));

      std::cout << " left." << std::endl;
    }
    else
    {
      std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::CirclesPointDetector(patternSize));
      originalImagesRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
      dynamic_cast<niftk::CirclesPointDetector*>(originalImagesRight.back().first.get())->SetImage(&(originalImagesRight.back().second));

      cv::Mat greyImageClone = greyImage.clone();
      std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::CirclesPointDetector(patternSize));
      imagesForWarpingRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
      dynamic_cast<niftk::CirclesPointDetector*>(imagesForWarpingRight.back().first.get())->SetImage(&(imagesForWarpingRight.back().second));

      std::cout << " right." << std::endl;
    }
  }

  std::cout << "originalImagesLeft.size()=" << originalImagesLeft.size() << std::endl;
  REQUIRE( originalImagesLeft.size() >= 1 );
  std::cout << "originalImagesRight.size()=" << originalImagesRight.size() << std::endl;
  REQUIRE( originalImagesRight.size() >= 1 );
  REQUIRE( originalImagesLeft.size()  == originalImagesRight.size());

  std::pair< cv::Mat, niftk::PointSet> referenceImageData;
  referenceImageData.first = referenceImage;
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
  cv::Mat leftToRightRotationMatrix;
  cv::Mat leftToRightTranslationVector;

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
        leftToRightRotationMatrix,
        leftToRightTranslationVector,
        essentialMatrix,
        fundamentalMatrix,
        flags
        );

  double tolerance = 0.005;

  cv::Mat rvec;
  cv::Rodrigues(leftToRightRotationMatrix, rvec);

  std::cout << "R1=" << rvec.at<double>(0,0) << std::endl;
  std::cout << "R2=" << rvec.at<double>(0,1) << std::endl;
  std::cout << "R3=" << rvec.at<double>(0,2) << std::endl;
  std::cout << "T1=" << leftToRightTranslationVector.at<double>(0,0) << std::endl;
  std::cout << "T2=" << leftToRightTranslationVector.at<double>(1,0) << std::endl;
  std::cout << "T3=" << leftToRightTranslationVector.at<double>(2,0) << std::endl;
  std::cout << "RMS=" << rms << std::endl;

  REQUIRE( fabs(rvec.at<double>(0,0) - eR1) < tolerance );
  REQUIRE( fabs(rvec.at<double>(0,1) - eR2) < tolerance );
  REQUIRE( fabs(rvec.at<double>(0,2) - eR3) < tolerance );
  REQUIRE( fabs(leftToRightTranslationVector.at<double>(0,0) - eT1) < tolerance );
  REQUIRE( fabs(leftToRightTranslationVector.at<double>(1,0) - eT2) < tolerance );
  REQUIRE( fabs(leftToRightTranslationVector.at<double>(2,0) - eT3) < tolerance );
}
