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
#include <niftkZhangCameraCalibration.h>
#include <niftkStereoCameraCalibration.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkIOUtilities.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>
#include <ostream>

TEST_CASE( "Stereo Chessboard", "[StereoCalibration]" ) {

  int expectedMinimumNumberOfArguments =  16;
  if (niftk::argc < expectedMinimumNumberOfArguments)
  {
    std::cerr << "Usage: niftkStereoChessboardCameraCalibrationTest modelFileName cornersInX cornersInY eR1 eR2 eR3 eT1 eT2 eT3 zeroDist rotTol transTol image1.png image2.png etc." << std::endl;
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
  int zeroDistortion = atoi(niftk::argv[10]);
  float rotationTolerance = atof(niftk::argv[11]);
  float translationTolerance = atof(niftk::argv[12]);

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

  // Loads "model"
  niftk::Model3D model = niftk::LoadModel3D(modelFileName);
  REQUIRE( model.size() == numberInternalCornersInY*numberInternalCornersInX );

  // Loads image data.
  cv::Size2i corners(numberInternalCornersInX, numberInternalCornersInY);

  cv::Size2i imageSize;
  niftk::PointSet pointSet;
  std::list<niftk::PointSet> listOfPointsLeft;
  std::list<niftk::PointSet> listOfPointsRight;

  std::vector<cv::Mat> colourLeftImages;
  std::vector<cv::Mat> colourRightImages;

  for (int i = 13; i < niftk::argc; i++)
  {
    cv::Mat image = cv::imread(niftk::argv[i]);
    if (image.rows > 0 && image.cols > 0)
    {
      cv::Mat greyImage;
      cv::cvtColor(image, greyImage, CV_BGR2GRAY);

      imageSize.width = greyImage.cols;
      imageSize.height = greyImage.rows;

      niftk::ChessboardPointDetector detector(corners);
      detector.SetImage(&greyImage);
      pointSet = detector.GetPoints();

      std::cout << "i=" << i << ", file=" << niftk::argv[i] << ", points=" << pointSet.size() << std::endl;

      if (pointSet.size() > 0)
      {
        if (i-13 < (niftk::argc-13)/2)
        {
          listOfPointsLeft.push_back(pointSet);
          std::cout << " left." << std::endl;

          colourLeftImages.push_back(image);
        }
        else
        {
          listOfPointsRight.push_back(pointSet);
          std::cout << " right." << std::endl;

          colourRightImages.push_back(image);
        }
      }
    }
  }

  std::cout << "listOfPointsLeft.size()=" << listOfPointsLeft.size() << std::endl;
  REQUIRE( listOfPointsLeft.size() >= 1 );
  std::cout << "listOfPointsRight.size()=" << listOfPointsRight.size() << std::endl;
  REQUIRE( listOfPointsRight.size() >= 1 );
  REQUIRE( listOfPointsLeft.size()  == listOfPointsRight.size());

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

  niftk::ZhangMonoCameraCalibration(model,
                                    listOfPointsLeft,
                                    imageSize,
                                    intrinsicLeft,
                                    distortionLeft,
                                    rvecsLeft,
                                    tvecsLeft,
                                    flags
                                   );

  niftk::ZhangMonoCameraCalibration(model,
                                    listOfPointsRight,
                                    imageSize,
                                    intrinsicRight,
                                    distortionRight,
                                    rvecsRight,
                                    tvecsRight,
                                    flags
                                   );

  cv::Matx21d result = niftk::StereoCameraCalibration(model,
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
                                                      leftToRightRotationMatrix,
                                                      leftToRightTranslationVector,
                                                      essentialMatrix,
                                                      fundamentalMatrix,
                                                      flags | CV_CALIB_USE_INTRINSIC_GUESS,
                                                      false, // just do optimisation of 2D reprojection error.
                                                      false,  // for stereo extrinsics, only do 2DOF
                                                      false   // also, simplify to perfect x translation and y rotation
                                                     );

  cv::Mat rvec;
  cv::Rodrigues(leftToRightRotationMatrix, rvec);


  REQUIRE( fabs(rvec.at<double>(0,0) - eR1) < rotationTolerance );
  REQUIRE( fabs(rvec.at<double>(0,1) - eR2) < rotationTolerance );
  REQUIRE( fabs(rvec.at<double>(0,2) - eR3) < rotationTolerance );
  REQUIRE( fabs(leftToRightTranslationVector.at<double>(0,0) - eT1) < translationTolerance );
  REQUIRE( fabs(leftToRightTranslationVector.at<double>(1,0) - eT2) < translationTolerance );
  REQUIRE( fabs(leftToRightTranslationVector.at<double>(2,0) - eT3) < translationTolerance );

  // Don't have a unit test yet.
  //niftk::SaveNifTKIntrinsics(intrinsicLeft, distortionLeft, "/tmp/calib.left.intrinsics.txt");
  //niftk::SaveNifTKIntrinsics(intrinsicRight, distortionRight, "/tmp/calib.right.intrinsics.txt");
  //niftk::SaveNifTKStereoExtrinsics(rightToLeftRotation, rightToLeftTranslation, "/tmp/calib.r2l.txt");
  //niftk::SaveRigidParams(rvec, rightToLeftTranslationVector, "/tmp/calib.r2l.param.txt");
  //cv::Matx44d mat = niftk::RodriguesToMatrix(rvec, rightToLeftTranslationVector);
  //niftk::SaveRigidParams(mat, "/tmp/calib.r2l.param2.txt");
}
