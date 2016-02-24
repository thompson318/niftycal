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
#include <iostream>
#include <list>

#include <cv.h>
#include <highgui.h>
#include <niftkOpenCVChessboardPointDetector.h>
#include <niftkStereoCameraCalibration.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkModel3D.h>

TEST_CASE( "Stereo Chessboard", "[StereoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  13;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkStereChessboardCameraCalibrationTest squareSizeInMillimetres cornersInX cornersInY eRMSLeft eRMSRight eR1 eR2 eR3 eT1 eT2 eT3 image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  float squareSizeInMillimetres = atof(niftk::argv[1]);
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

  // Generates "model"
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

      niftk::OpenCVChessboardPointDetector detector(&greyImage, corners);
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

  cv::Mat intrinsicLeft = cvCreateMat (3,3,CV_64FC1);
  cv::Mat distortionLeft = cvCreateMat (1,4,CV_64FC1);
  std::vector<cv::Mat> rvecsLeft;
  std::vector<cv::Mat> tvecsLeft;

  cv::Mat intrinsicRight = cvCreateMat (3,3,CV_64FC1);
  cv::Mat distortionRight = cvCreateMat (1,4,CV_64FC1);
  std::vector<cv::Mat> rvecsRight;
  std::vector<cv::Mat> tvecsRight;

  cv::Mat essentialMatrix = cvCreateMat (3,3,CV_64FC1);
  cv::Mat fundamentalMatrix = cvCreateMat (3,3,CV_64FC1);
  cv::Mat left2RightRotation = cvCreateMat (1,3,CV_64FC1);
  cv::Mat left2RightTranslation = cvCreateMat (1,3,CV_64FC1);

  niftk::StereoCameraCalibration(model,
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
                                 left2RightRotation,
                                 left2RightTranslation,
                                 essentialMatrix,
                                 fundamentalMatrix
                                );

  std::cout << "R1=" << left2RightRotation.at<double>(0,0) << std::endl;
  std::cout << "R2=" << left2RightRotation.at<double>(0,1) << std::endl;
  std::cout << "R3=" << left2RightRotation.at<double>(0,2) << std::endl;
  std::cout << "T1=" << left2RightTranslation.at<double>(0,0) << std::endl;
  std::cout << "T2=" << left2RightTranslation.at<double>(0,1) << std::endl;
  std::cout << "T3=" << left2RightTranslation.at<double>(0,2) << std::endl;

  double tolerance = 0.005;
  REQUIRE( fabs(left2RightRotation.at<double>(0,0) - eR1) < tolerance );
  REQUIRE( fabs(left2RightRotation.at<double>(0,1) - eR2) < tolerance );
  REQUIRE( fabs(left2RightRotation.at<double>(0,2) - eR3) < tolerance );
  REQUIRE( fabs(left2RightTranslation.at<double>(0,0) - eT1) < tolerance );
  REQUIRE( fabs(left2RightTranslation.at<double>(0,1) - eT2) < tolerance );
  REQUIRE( fabs(left2RightTranslation.at<double>(0,2) - eT3) < tolerance );
}
