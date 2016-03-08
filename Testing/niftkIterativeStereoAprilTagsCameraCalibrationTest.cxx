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
#include <niftkAprilTagsPointDetector.h>
#include <niftkIterativeStereoCameraCalibration.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkIOUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>

TEST_CASE( "Iterative Stereo AprilTags", "[StereoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  13;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkIterativeStereoAprilTagsCameraCalibrationTest modelImage modelFileName fileOfPoints eRMSLeft eRMSRight eR1 eR2 eR3 eT1 eT2 eT3 image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  std::string modelImage = niftk::argv[1];
  std::string modelFileName = niftk::argv[2];
  std::string fileOfReferencePoints = niftk::argv[3];
  int zeroDistortion = atoi(niftk::argv[4]);
  float eR1 = atof(niftk::argv[5]);
  float eR2 = atof(niftk::argv[6]);
  float eR3 = atof(niftk::argv[7]);
  float eT1 = atof(niftk::argv[8]);
  float eT2 = atof(niftk::argv[9]);
  float eT3 = atof(niftk::argv[10]);

  // Should have an even number of images left.
  if ((niftk::argc - 11) % 2 != 0)
  {
    niftkNiftyCalThrow() << "Should have an even number of images.";
  }

  cv::Mat referenceImage = cv::imread(modelImage);

  // Loads "model"
  niftk::Model3D model = niftk::LoadModel3D(modelFileName);

  // Loads image data.
  cv::Size2i imageSize;

  // Loads all image data.
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImagesLeft;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarpingLeft;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImagesRight;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarpingRight;

  for (int i = 11; i < niftk::argc; i++)
  {
    cv::Mat image = cv::imread(niftk::argv[i]);
    imageSize.width = image.cols;
    imageSize.height = image.rows;

    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);

    std::cout << "i=" << i << ", file=" << niftk::argv[i] << std::endl;

    if (i-11 < (niftk::argc-11)/2)
    {
      std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::AprilTagsPointDetector(true, // do include corners
                                                                                                  "36h11",
                                                                                                  0,
                                                                                                  0.8));
      originalImagesLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
      dynamic_cast<niftk::AprilTagsPointDetector*>(originalImagesLeft.back().first.get())->SetImage(&(originalImagesLeft.back().second));

      cv::Mat greyImageClone = greyImage.clone();
      std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::AprilTagsPointDetector(true, // do include corners
                                                                                                "36h11",
                                                                                                0,
                                                                                                0.8));
      imagesForWarpingLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
      dynamic_cast<niftk::AprilTagsPointDetector*>(imagesForWarpingLeft.back().first.get())->SetImage(&(imagesForWarpingLeft.back().second));

      std::cout << " left." << std::endl;
    }
    else
    {
      std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::AprilTagsPointDetector(true, // do include corners
                                                                                                  "36h11",
                                                                                                  0,
                                                                                                  0.8));
      originalImagesRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
      dynamic_cast<niftk::AprilTagsPointDetector*>(originalImagesRight.back().first.get())->SetImage(&(originalImagesRight.back().second));

      cv::Mat greyImageClone = greyImage.clone();
      std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::AprilTagsPointDetector(true, // do include corners
                                                                                                "36h11",
                                                                                                0,
                                                                                                0.8));
      imagesForWarpingRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
      dynamic_cast<niftk::AprilTagsPointDetector*>(imagesForWarpingRight.back().first.get())->SetImage(&(imagesForWarpingRight.back().second));

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
