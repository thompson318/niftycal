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
#include <niftkRenderingCameraCalibration.h>
#include <niftkCalibratedRenderingPipeline.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <list>
#include <ostream>
#include <QApplication>
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>

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
                                                      false // just do optimisation of 2D reprojection error.
                                                     );

  QApplication app(niftk::argc, niftk::argv);

  QVTKWidget *widget = new QVTKWidget();
  widget->show();
  widget->resize(imageSize.width, imageSize.height);

  vtkRenderWindow *window = widget->GetRenderWindow();
  window->DoubleBufferOff();
  window->GetInteractor()->Disable();

  niftk::CalibratedRenderingPipeline *p = new niftk::CalibratedRenderingPipeline(imageSize, imageSize,
                                                                                 "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                                                                 "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
                                                                                 true);
  p->ConnectToRenderWindow(window);

  // Dump rhs, video+rendering for all views.
  for (int i = 0; i < colourRightImages.size(); i++)
  {
    std::ostringstream videoFileName;
    videoFileName << "/tmp/matt.rhs.normal.video." << i << ".png";

    cv::Mat mapX = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
    cv::Mat mapY = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
    cv::initUndistortRectifyMap(intrinsicRight, distortionRight, cv::noArray(), intrinsicRight, imageSize, CV_32FC1, mapX, mapY);

    cv::Mat undistorted;
    cv::remap(colourRightImages[i], undistorted, mapX, mapY, CV_INTER_LANCZOS4);

    cv::imwrite(videoFileName.str(), undistorted);

    cv::Matx44d cameraMatrix = niftk::RodriguesToMatrix(rvecsRight[i], tvecsRight[i]);

    std::ostringstream renderedFileName;
    renderedFileName << "/tmp/matt.rhs.normal.rendering." << i << ".png";

    p->SetBackgroundColour(0, 0, 255);
    p->SetIntrinsics(intrinsicRight);
    p->SetWorldToCameraMatrix(cameraMatrix);
    p->DumpScreen(renderedFileName.str());
  }

  delete p;

  std::list<niftk::PointSet> listOfPointsOnUndistortedLeft;
  std::list<niftk::PointSet> listOfPointsOnUndistortedRight;

  for (int i = 0; i < colourLeftImages.size(); i++)
  {
    cv::Mat greyLeft;
    cv::cvtColor(colourLeftImages[i], greyLeft, CV_BGR2GRAY);

    cv::Mat mapX = cvCreateMat(colourLeftImages[i].rows, colourLeftImages[i].cols, CV_32FC1);
    cv::Mat mapY = cvCreateMat(colourLeftImages[i].rows, colourLeftImages[i].cols, CV_32FC1);
    cv::initUndistortRectifyMap(intrinsicLeft, distortionLeft, cv::noArray(), intrinsicLeft, imageSize, CV_32FC1, mapX, mapY);

    cv::Mat undistorted;
    cv::remap(greyLeft, undistorted, mapX, mapY, CV_INTER_LANCZOS4);

    niftk::ChessboardPointDetector detector(corners);
    detector.SetImage(&undistorted);
    pointSet = detector.GetPoints();
    if (pointSet.size() != numberInternalCornersInX * numberInternalCornersInY)
    {
      niftkNiftyCalThrow() << "Failed to extract left points, i=" << i;
    }
    listOfPointsOnUndistortedLeft.push_back(pointSet);
  }

  for (int i = 0; i < colourRightImages.size(); i++)
  {
    cv::Mat greyRight;
    cv::cvtColor(colourRightImages[i], greyRight, CV_BGR2GRAY);

    cv::Mat mapX = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
    cv::Mat mapY = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
    cv::initUndistortRectifyMap(intrinsicRight, distortionRight, cv::noArray(), intrinsicRight, imageSize, CV_32FC1, mapX, mapY);

    cv::Mat undistorted;
    cv::remap(greyRight, undistorted, mapX, mapY, CV_INTER_LANCZOS4);

    niftk::ChessboardPointDetector detector(corners);
    detector.SetImage(&undistorted);
    pointSet = detector.GetPoints();
    if (pointSet.size() != numberInternalCornersInX * numberInternalCornersInY)
    {
      niftkNiftyCalThrow() << "Failed to extract right points, i=" << i;
    }

    listOfPointsOnUndistortedRight.push_back(pointSet);
  }

  // Recompute RMS error
  cv::Mat zeroDistortionParams = cv::Mat::zeros(1, 5, CV_64FC1);

  cv::Point3d rmsPerAxis;
  double rmsAgain = niftk::ComputeRMSReconstructionError(model,
                                                         listOfPointsOnUndistortedLeft,
                                                         listOfPointsOnUndistortedRight,
                                                         intrinsicLeft,
                                                         zeroDistortionParams,
                                                         rvecsLeft,
                                                         tvecsLeft,
                                                         intrinsicRight,
                                                         zeroDistortionParams,
                                                         leftToRightRotationMatrix,
                                                         leftToRightTranslationVector,
                                                         rmsPerAxis
                                                         );

  std::cout << "Stereo RMS-2D=" << result(0, 0) << std::endl;
  std::cout << "Stereo RMS-3D=" << result(1, 0) << std::endl;
  std::cout << "Stereo RMS-3D=" << rmsAgain << std::endl;
  std::cout << "Stereo RMS-3Dx=" << rmsPerAxis.x << std::endl;
  std::cout << "Stereo RMS-3Dy=" << rmsPerAxis.y << std::endl;
  std::cout << "Stereo RMS-3Dz=" << rmsPerAxis.z << std::endl;

  niftk::RenderingStereoCameraCalibration(window,
                                          imageSize,
                                          imageSize,
                                          "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                          "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
                                          colourLeftImages,
                                          colourRightImages,
                                          intrinsicLeft,
                                          distortionLeft,
                                          rvecsLeft,
                                          tvecsLeft,
                                          intrinsicRight,
                                          distortionRight,
                                          rvecsRight,
                                          tvecsRight,
                                          leftToRightRotationMatrix,
                                          leftToRightTranslationVector
                                         );

  p = new niftk::CalibratedRenderingPipeline(imageSize, imageSize,
                                             "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                             "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
                                             true);
  p->ConnectToRenderWindow(window);

  // Dump rhs, video+rendering for all views.
  for (int i = 0; i < colourRightImages.size(); i++)
  {
    std::ostringstream videoFileName;
    videoFileName << "/tmp/matt.rhs.optimised.video." << i << ".png";

    cv::Mat mapX = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
    cv::Mat mapY = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
    cv::initUndistortRectifyMap(intrinsicRight, distortionRight, cv::noArray(), intrinsicRight, imageSize, CV_32FC1, mapX, mapY);

    cv::Mat undistorted;
    cv::remap(colourRightImages[i], undistorted, mapX, mapY, CV_INTER_LANCZOS4);

    cv::imwrite(videoFileName.str(), undistorted);

    cv::Matx44d cameraMatrix = niftk::RodriguesToMatrix(rvecsRight[i], tvecsRight[i]);

    std::ostringstream renderedFileName;
    renderedFileName << "/tmp/matt.rhs.optimised.rendering." << i << ".png";

    p->SetBackgroundColour(0, 0, 255);
    p->SetIntrinsics(intrinsicRight);
    p->SetWorldToCameraMatrix(cameraMatrix);
    p->DumpScreen(renderedFileName.str());
  }

  delete p;

  std::vector<cv::Mat> optimisedRVecLeft;
  std::vector<cv::Mat> optimisedTVecLeft;

  std::list<niftk::PointSet> listOfPointsOnUndistortedOptimisedLeft;
  std::list<niftk::PointSet> listOfPointsOnUndistortedOptimisedRight;
  niftk::PointSet pointSetLeft;
  niftk::PointSet pointSetRight;

  for (int i = 0; i < colourLeftImages.size(); i++)
  {
    cv::Mat greyLeft;
    cv::cvtColor(colourLeftImages[i], greyLeft, CV_BGR2GRAY);

    cv::Mat mapX = cvCreateMat(colourLeftImages[i].rows, colourLeftImages[i].cols, CV_32FC1);
    cv::Mat mapY = cvCreateMat(colourLeftImages[i].rows, colourLeftImages[i].cols, CV_32FC1);
    cv::initUndistortRectifyMap(intrinsicLeft, distortionLeft, cv::noArray(), intrinsicLeft, imageSize, CV_32FC1, mapX, mapY);

    cv::Mat undistortedLeft;
    cv::remap(greyLeft, undistortedLeft, mapX, mapY, CV_INTER_LANCZOS4);

    niftk::ChessboardPointDetector leftDetector(corners);
    leftDetector.SetImage(&undistortedLeft);
    pointSetLeft = leftDetector.GetPoints();

    cv::Mat greyRight;
    cv::cvtColor(colourRightImages[i], greyRight, CV_BGR2GRAY);

    mapX = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
    mapY = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
    cv::initUndistortRectifyMap(intrinsicRight, distortionRight, cv::noArray(), intrinsicRight, imageSize, CV_32FC1, mapX, mapY);

    cv::Mat undistortedRight;
    cv::remap(greyRight, undistortedRight, mapX, mapY, CV_INTER_LANCZOS4);

    niftk::ChessboardPointDetector detectorRight(corners);
    detectorRight.SetImage(&undistortedRight);
    pointSetRight = detectorRight.GetPoints();

    if (   pointSetLeft.size() == numberInternalCornersInX * numberInternalCornersInY
        && pointSetRight.size() == numberInternalCornersInX * numberInternalCornersInY
        )
    {
      listOfPointsOnUndistortedOptimisedLeft.push_back(pointSetLeft);
      listOfPointsOnUndistortedOptimisedRight.push_back(pointSetRight);
      optimisedRVecLeft.push_back(rvecsLeft[i]);
      optimisedTVecLeft.push_back(tvecsLeft[i]);
    }
    else
    {
      std::cerr << "Dropping frame i=" << i << std::endl;
    }
  }

  // Recompute RMS error
  rmsAgain = niftk::ComputeRMSReconstructionError(model,
                                                  listOfPointsOnUndistortedOptimisedLeft,
                                                  listOfPointsOnUndistortedOptimisedRight,
                                                  intrinsicLeft,
                                                  zeroDistortionParams,
                                                  optimisedRVecLeft,
                                                  optimisedTVecLeft,
                                                  intrinsicRight,
                                                  zeroDistortionParams,
                                                  leftToRightRotationMatrix,
                                                  leftToRightTranslationVector,
                                                  rmsPerAxis
                                                  );

  cv::Mat rvec;
  cv::Rodrigues(leftToRightRotationMatrix, rvec);

  std::cout << "Stereo RMS-(Rendered)-3D=" << rmsAgain << std::endl;
  std::cout << "Stereo RMS-(Rendered)-3Dx=" << rmsPerAxis.x << std::endl;
  std::cout << "Stereo RMS-(Rendered)-3Dy=" << rmsPerAxis.y << std::endl;
  std::cout << "Stereo RMS-(Rendered)-3Dz=" << rmsPerAxis.z << std::endl;
  std::cout << "Stereo R1=" << rvec.at<double>(0,0) << std::endl;
  std::cout << "Stereo R2=" << rvec.at<double>(0,1) << std::endl;
  std::cout << "Stereo R3=" << rvec.at<double>(0,2) << std::endl;
  std::cout << "Stereo T1=" << leftToRightTranslationVector.at<double>(0,0) << std::endl;
  std::cout << "Stereo T2=" << leftToRightTranslationVector.at<double>(1,0) << std::endl;
  std::cout << "Stereo T3=" << leftToRightTranslationVector.at<double>(2,0) << std::endl;
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
