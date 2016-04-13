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
#include <niftkAprilTagsPointDetector.h>
#include <niftkIterativeMonoCameraCalibration.h>
#include <niftkIOUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>

TEST_CASE( "Iterative Mono AprilTags", "[MonoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  18;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkIterativeMonoAprilTagsCameraCalibrationTest modelImage modelFileName fileOfPoints zeroDist scaleX scaleY eRMS eFx eFy eCx eCy eK1 eK2 eP1 eP2 image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  std::string modelImage = niftk::argv[1];
  std::string modelFileName = niftk::argv[2];
  std::string fileOfReferencePoints = niftk::argv[3];
  int zeroDistortion = atoi(niftk::argv[4]);
  float scaleX = atof(niftk::argv[5]);
  float scaleY = atof(niftk::argv[6]);
  float eRMS = atof(niftk::argv[7]);
  float eFx = atof(niftk::argv[8]);
  float eFy = atof(niftk::argv[9]);
  float eCx = atof(niftk::argv[10]);
  float eCy = atof(niftk::argv[11]);
  float eK1 = atof(niftk::argv[12]);
  float eK2 = atof(niftk::argv[13]);
  float eP1 = atof(niftk::argv[14]);
  float eP2 = atof(niftk::argv[15]);

  cv::Mat referenceImage = cv::imread(modelImage);
  cv::Mat referenceImageGreyScale;
  cv::cvtColor(referenceImage, referenceImageGreyScale, CV_BGR2GRAY);

  // Loads "model"
  niftk::Model3D model = niftk::LoadModel3D(modelFileName);

  // Loads all image data.
  cv::Size2i imageSize;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImages;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarping;
  for (int i = 17; i < niftk::argc; i++)
  {
    cv::Mat image = cv::imread(niftk::argv[i]);
    imageSize.width = image.cols;
    imageSize.height = image.rows;

    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);

    std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::AprilTagsPointDetector(true, // do include corners
                                                                                                "36h11",
                                                                                                0,
                                                                                                0.8));
    originalImages.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
    dynamic_cast<niftk::AprilTagsPointDetector*>(originalImages.back().first.get())->SetImage(&(originalImages.back().second));

    cv::Mat greyImageClone = greyImage.clone();
    std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::AprilTagsPointDetector(true, // do include corners
                                                                                              "36h11",
                                                                                              0,
                                                                                              0.8));
    imagesForWarping.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
    dynamic_cast<niftk::AprilTagsPointDetector*>(imagesForWarping.back().first.get())->SetImage(&(imagesForWarping.back().second));
  }

  REQUIRE(originalImages.size() == niftk::argc-17);
  REQUIRE(imagesForWarping.size() == niftk::argc-17);

  cv::Mat intrinsic;
  cv::Mat distortion;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
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

  double rms = niftk::IterativeMonoCameraCalibration(
        model,
        referenceImageData,
        originalImages,
        imagesForWarping,
        imageSize,
        intrinsic,
        distortion,
        rvecs,
        tvecs,
        flags
        );

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
