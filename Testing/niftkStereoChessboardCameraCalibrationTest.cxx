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
#include <niftkMonoCameraCalibration.h>
#include <niftkStereoCameraCalibration.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkIOUtilities.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>

TEST_CASE( "Stereo Chessboard", "[StereoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  13;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkStereChessboardCameraCalibrationTest modelFileName cornersInX cornersInY eRMSLeft eRMSRight eR1 eR2 eR3 eT1 eT2 eT3 image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  std::string modelFileName = niftk::argv[1];
  int numberInternalCornersInX = atoi(niftk::argv[2]);
  int numberInternalCornersInY = atoi(niftk::argv[3]);
  float eR1 = atof(niftk::argv[4]);
  float eR2 = atof(niftk::argv[5]);
  float eR3 = atof(niftk::argv[6]);
  float eT1 = atof(niftk::argv[7]);
  float eT2 = atof(niftk::argv[8]);
  float eT3 = atof(niftk::argv[9]);

  if (numberInternalCornersInX < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInX < 2.";
  }
  if (numberInternalCornersInY < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInY < 2.";
  }

  // Should have an even number of images left.
  if ((niftk::argc - 10) % 2 != 0)
  {
    niftkNiftyCalThrow() << "Should have an even number of images.";
  }

  // Loads "model"
  niftk::Model3D model = niftk::LoadModel3D(modelFileName);
  REQUIRE( model.size() == numberInternalCornersInY*numberInternalCornersInX );

  // Loads image data.
  cv::Size2i corners(numberInternalCornersInX, numberInternalCornersInY);

  cv::Size2i imageSize;
  niftk::PointSet pointSet;
  std::list<niftk::PointSet> listOfPointsLeft;
  std::list<niftk::PointSet> listOfPointsRight;

  for (int i = 10; i < niftk::argc; i++)
  {
    cv::Mat image = cv::imread(niftk::argv[i]);
    if (image.rows > 0 && image.cols > 0)
    {
      cv::Mat greyImage;
      cv::cvtColor(image, greyImage, CV_BGR2GRAY);

      imageSize.width = greyImage.cols;
      imageSize.height = greyImage.rows;

      niftk::OpenCVChessboardPointDetector detector(corners);
      detector.SetImage(&greyImage);
      pointSet = detector.GetPoints();

      std::cout << "i=" << i << ", file=" << niftk::argv[i] << ", points=" << pointSet.size();

      if (pointSet.size() > 0)
      {
        if (i-10 < (niftk::argc-10)/2)
        {
          listOfPointsLeft.push_back(pointSet);
          std::cout << " left." << std::endl;
        }
        else
        {
          listOfPointsRight.push_back(pointSet);
          std::cout << " right." << std::endl;
        }
      }
    }
  }

  std::cout << "listOfPointsLeft.size()=" << listOfPointsLeft.size() << std::endl;
  REQUIRE( listOfPointsLeft.size() >= 1 );
  std::cout << "listOfPointsRight.size()=" << listOfPointsRight.size() << std::endl;
  REQUIRE( listOfPointsRight.size() >= 1 );
  REQUIRE( listOfPointsLeft.size()  == listOfPointsRight.size());

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
  cv::Mat rightToLeftRotation;
  cv::Mat rightToLeftTranslation;

  niftk::MonoCameraCalibration(model,
                               listOfPointsLeft,
                               imageSize,
                               intrinsicLeft,
                               distortionLeft,
                               rvecsLeft,
                               tvecsLeft
                              );

  niftk::MonoCameraCalibration(model,
                               listOfPointsRight,
                               imageSize,
                               intrinsicRight,
                               distortionRight,
                               rvecsRight,
                               tvecsRight
                              );

  double rms = niftk::StereoCameraCalibration(model,
                                              listOfPointsLeft,
                                              listOfPointsRight,
                                              imageSize,
                                              intrinsicLeft,
                                              distortionLeft,
                                              rvecsLeft,
                                              tvecsLeft,
                                              intrinsicRight,
                                              distortionRight,
                                              rvecsRight,
                                              tvecsRight,
                                              rightToLeftRotation,
                                              rightToLeftTranslation,
                                              essentialMatrix,
                                              fundamentalMatrix,
                                              CV_CALIB_USE_INTRINSIC_GUESS
                                             );

  std::cout << "Stereo RMS=" << rms << std::endl;
  std::cout << "Stereo R1=" << rightToLeftRotation.at<double>(0,0) << std::endl;
  std::cout << "Stereo R2=" << rightToLeftRotation.at<double>(0,1) << std::endl;
  std::cout << "Stereo R3=" << rightToLeftRotation.at<double>(0,2) << std::endl;
  std::cout << "Stereo T1=" << rightToLeftTranslation.at<double>(0,0) << std::endl;
  std::cout << "Stereo T2=" << rightToLeftTranslation.at<double>(0,1) << std::endl;
  std::cout << "Stereo T3=" << rightToLeftTranslation.at<double>(0,2) << std::endl;
  std::cout << "Stereo Fxl=" << intrinsicLeft.at<double>(0,0) << std::endl;
  std::cout << "Stereo Fyl=" << intrinsicLeft.at<double>(1,1) << std::endl;
  std::cout << "Stereo Cxl=" << intrinsicLeft.at<double>(0,2) << std::endl;
  std::cout << "Stereo Cyl=" << intrinsicLeft.at<double>(1,2) << std::endl;
  std::cout << "Stereo K1l=" << distortionLeft.at<double>(0,0) << std::endl;
  std::cout << "Stereo K2l=" << distortionLeft.at<double>(0,1) << std::endl;
  std::cout << "Stereo P1l=" << distortionLeft.at<double>(0,2) << std::endl;
  std::cout << "Stereo P2l=" << distortionLeft.at<double>(0,3) << std::endl;
  std::cout << "Stereo Fxr=" << intrinsicRight.at<double>(0,0) << std::endl;
  std::cout << "Stereo Fyr=" << intrinsicRight.at<double>(1,1) << std::endl;
  std::cout << "Stereo Cxr=" << intrinsicRight.at<double>(0,2) << std::endl;
  std::cout << "Stereo Cyr=" << intrinsicRight.at<double>(1,2) << std::endl;
  std::cout << "Stereo K1r=" << distortionRight.at<double>(0,0) << std::endl;
  std::cout << "Stereo K2r=" << distortionRight.at<double>(0,1) << std::endl;
  std::cout << "Stereo P1r=" << distortionRight.at<double>(0,2) << std::endl;
  std::cout << "Stereo P2r=" << distortionRight.at<double>(0,3) << std::endl;

  double tolerance = 0.005;
  REQUIRE( fabs(rightToLeftRotation.at<double>(0,0) - eR1) < tolerance );
  REQUIRE( fabs(rightToLeftRotation.at<double>(0,1) - eR2) < tolerance );
  REQUIRE( fabs(rightToLeftRotation.at<double>(0,2) - eR3) < tolerance );
  REQUIRE( fabs(rightToLeftTranslation.at<double>(0,0) - eT1) < tolerance );
  REQUIRE( fabs(rightToLeftTranslation.at<double>(0,1) - eT2) < tolerance );
  REQUIRE( fabs(rightToLeftTranslation.at<double>(0,2) - eT3) < tolerance );

  // Don't have a unit test yet.
  //niftk::SaveNifTKIntrinsics(intrinsicLeft, distortionLeft, "/tmp/calib.left.intrinsics.txt");
  //niftk::SaveNifTKIntrinsics(intrinsicRight, distortionRight, "/tmp/calib.right.intrinsics.txt");
  //niftk::SaveNifTKStereoExtrinsics(rightToLeftRotation, rightToLeftTranslation, "/tmp/calib.r2l.txt");

}
