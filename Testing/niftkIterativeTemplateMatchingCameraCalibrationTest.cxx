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
#include <niftkRingsPointDetector.h>
#include <niftkCirclesIterativePointDetector.h>
#include <niftkIterativeMonoCameraCalibration.h>
#include <niftkIOUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>

TEST_CASE( "Iterative Template Matching (Rings/Circles)", "[MonoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  20;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkIterativeMonoTemplateMatchingCameraCalibrationTest method referenceImage referenceModelFileName referencePointsFileName templateImageFileName ringsInX ringsInY  eRMS eFx eFy eCx eCy eK1 eK2 eP1 eP2 image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  int calibrationMethod = atoi(niftk::argv[1]);
  std::string referenceImageFileName = niftk::argv[2];
  std::string referenceModelFileName = niftk::argv[3];
  std::string referencePointsFileName = niftk::argv[4];
  std::string templateFileName = niftk::argv[5];
  int ringsInX = atoi(niftk::argv[6]);
  int ringsInY = atoi(niftk::argv[7]);
  int zeroDistortion = atoi(niftk::argv[8]);
  float eRMS = atof(niftk::argv[9]);
  float eFx = atof(niftk::argv[10]);
  float eFy = atof(niftk::argv[11]);
  float eCx = atof(niftk::argv[12]);
  float eCy = atof(niftk::argv[13]);
  float eK1 = atof(niftk::argv[14]);
  float eK2 = atof(niftk::argv[15]);
  float eP1 = atof(niftk::argv[16]);
  float eP2 = atof(niftk::argv[17]);

  if (ringsInX < 2)
  {
    niftkNiftyCalThrow() << "ringsInX < 2";
  }
  if (ringsInY < 2)
  {
    niftkNiftyCalThrow() << "ringsInY < 2";
  }

  cv::Mat referenceImage = cv::imread(referenceImageFileName);
  cv::Mat referenceImageGreyScale;
  cv::cvtColor(referenceImage, referenceImageGreyScale, CV_BGR2GRAY);

  niftk::Model3D model = niftk::LoadModel3D(referenceModelFileName);
  REQUIRE( model.size() == ringsInX*ringsInY );

  std::pair< cv::Mat, niftk::PointSet> referenceImageData;
  referenceImageData.first = referenceImageGreyScale;
  referenceImageData.second = niftk::LoadPointSet(referencePointsFileName);

  cv::Mat templateImage = cv::imread(templateFileName);
  cv::Mat templateImageGreyScale;
  cv::cvtColor(templateImage, templateImageGreyScale, CV_BGR2GRAY);

  cv::Size2i offset(10, 10);
  cv::Size2i rings(ringsInY, ringsInX);
  cv::Size2i imageSize;

  // Loads all image data.
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImages;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarping;
  for (int i = 18; i < niftk::argc; i++)
  {
    cv::Mat image = cv::imread(niftk::argv[i]);
    imageSize.width = image.cols;
    imageSize.height = image.rows;

    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);

    cv::Mat greyImageClone = greyImage.clone();
    unsigned long int maxArea = 10000;

    if (calibrationMethod == 0)
    {
      std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::RingsPointDetector(rings, offset));
      originalImages.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));

      std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::RingsPointDetector(rings, offset));
      imagesForWarping.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
    }
    else if (calibrationMethod == 1)
    {
      std::shared_ptr<niftk::IPoint2DDetector> originalDetector(new niftk::CirclesIterativePointDetector(rings, offset));
      originalImages.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));

      std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(new niftk::CirclesIterativePointDetector(rings, offset));
      imagesForWarping.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
    }
    else
    {
      niftkNiftyCalThrow() << "Invalid calibration method.";
    }

    niftk::TemplateMatchingPointDetector* opd =
        dynamic_cast<niftk::TemplateMatchingPointDetector*>(originalImages.back().first.get());

    opd->SetImage(&(originalImages.back().second));
    opd->SetTemplateImage(&templateImageGreyScale);
    opd->SetReferenceImage(&referenceImageGreyScale);
    opd->SetReferencePoints(referenceImageData.second);
    opd->SetMaxAreaInPixels(maxArea);
    opd->SetUseContours(true);
    opd->SetUseInternalResampling(false);
    opd->SetUseTemplateMatching(false);

    niftk::PointSet points = opd->GetPoints();
    REQUIRE(points.size() == ringsInX*ringsInY);


    niftk::TemplateMatchingPointDetector* cpd =
        dynamic_cast<niftk::TemplateMatchingPointDetector*>(imagesForWarping.back().first.get());

    cpd->SetImage(&(imagesForWarping.back().second));
    cpd->SetTemplateImage(&templateImageGreyScale);
    cpd->SetReferenceImage(&referenceImageGreyScale);
    cpd->SetReferencePoints(referenceImageData.second);
    cpd->SetMaxAreaInPixels(maxArea);
    cpd->SetUseContours(false);
    cpd->SetUseInternalResampling(false);
    cpd->SetUseTemplateMatching(true);
    cpd->SetInitialGuess(referenceImageData.second);
  }

  REQUIRE(originalImages.size() == niftk::argc-18);
  REQUIRE(imagesForWarping.size() == niftk::argc-18);

  cv::Mat intrinsic;
  cv::Mat distortion;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

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
