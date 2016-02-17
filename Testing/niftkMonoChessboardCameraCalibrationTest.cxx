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
#include <niftkMonoCameraCalibration.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkModel3D.h>

TEST_CASE( "Mono Chessboard", "[MonoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  6;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkMonoChessboardCameraCalibrationTest squareSizeInMillimetres cornersInX cornersInY image1.png image2.png etc." << std::endl;
    REQUIRE( niftk::argc >= expectedMinimumNumberOfArguments);
  }

  float squareSizeInMillimetres = atof(niftk::argv[1]);
  int numberInternalCornersInX = atoi(niftk::argv[2]);
  int numberInternalCornersInY = atoi(niftk::argv[3]);

  if (numberInternalCornersInX < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInX < 2";
  }
  if (numberInternalCornersInY < 2)
  {
    niftkNiftyCalThrow() << "numberInternalCornersInY < 2";
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

  niftk::PointSet pointSet;
  std::list<niftk::PointSet> listOfPoints;
  std::list<cv::Matx44d> extrinsics;

  for (int i = 4; i < niftk::argc; i++)
  {
    cv::Mat image = cv::imread(niftk::argv[i]);
    if (image.rows > 0 && image.cols > 0)
    {
      cv::Mat greyImage;
      cv::cvtColor(image, greyImage, CV_BGR2GRAY);

      niftk::OpenCVChessboardPointDetector detector(&greyImage, corners);
      pointSet = detector.GetPoints();

      std::cout << "i=" << i << ", file=" << niftk::argv[i] << ", points=" << pointSet.size() << std::endl;

      if (pointSet.size() > 0)
      {
        listOfPoints.push_back(pointSet);
      }

      cv::Matx44d ext;
      ext = cv::Matx44d::eye();

      extrinsics.push_back(ext);
    }
  }

  std::cout << "listOfPoints.size()=" << listOfPoints.size() << std::endl;
  REQUIRE( listOfPoints.size() >= 2 );

  cv::Matx33d intrinsic;
  cv::Matx16d distortion;

  double rms = niftk::MonoCameraCalibration(model,
                                            listOfPoints,
                                            intrinsic,
                                            distortion,
                                            extrinsics
                                            );
  REQUIRE( rms == 0 );
}
