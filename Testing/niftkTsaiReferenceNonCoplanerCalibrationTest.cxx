/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "catch.hpp"
#include <niftkCatchMain.h>
#include <niftkNiftyCalTypes.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkTsaiCameraCalibration.h>
#include <iostream>
#include <fstream>

TEST_CASE( "Tsai mono, reference case", "[mono]" ) {

  int expectedNumberOfArguments =  12;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkTsaiReferenceNonCoplanerCalibrationTest data.txt nx ny dx dy fx fy cx cy distortion rms" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  std::string dataFileName = niftk::argv[1];
  int nx = atoi(niftk::argv[2]);
  int ny = atoi(niftk::argv[3]);
  float dx = atof(niftk::argv[4]);
  float dy = atof(niftk::argv[5]);
  float eFx = atof(niftk::argv[6]);
  float eFy = atof(niftk::argv[7]);
  float eCx = atof(niftk::argv[8]);
  float eCy = atof(niftk::argv[9]);
  float dist = atof(niftk::argv[10]);
  float expectedRMS = atof(niftk::argv[11]);

  // Load reference data
  std::ifstream ifs;
  ifs.open (dataFileName, std::ofstream::in);
  if (!ifs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << dataFileName << " for reading.";
  }

  niftk::PointSet points;
  niftk::Model3D model;

  unsigned int pointCounter = 0;
  while (!ifs.eof())
  {
    niftk::Point2D p2;
    niftk::Point3D p3;

    ifs >> p3.point.x;
    ifs >> p3.point.y;
    ifs >> p3.point.z;

    ifs >> p2.point.x;
    ifs >> p2.point.y;

    if (!ifs.bad() && !ifs.fail())
    {

      p2.id = pointCounter;
      p3.id = pointCounter;

      points.insert(niftk::IdPoint2D(pointCounter, p2));
      model.insert(niftk::IdPoint3D(pointCounter, p3));

      pointCounter++;
    }
  }

  ifs.close();

  cv::Mat intrinsic;
  cv::Mat distortion;
  cv::Mat rvec;
  cv::Mat tvec;

  cv::Size imageSize(nx, ny);

  cv::Point2d sensorDimensions;
  sensorDimensions.x = dx;
  sensorDimensions.y = dy;

  double rms = niftk::TsaiMonoCameraCalibration(model, points, imageSize, sensorDimensions, intrinsic, distortion, rvec, tvec, true);

  std::cout << "RMS=" << rms << std::endl;
  std::cout << "Fx=" << intrinsic.at<double>(0,0) << std::endl;
  std::cout << "Fy=" << intrinsic.at<double>(1,1) << std::endl;
  std::cout << "Cx=" << intrinsic.at<double>(0,2) << std::endl;
  std::cout << "Cy=" << intrinsic.at<double>(1,2) << std::endl;
  std::cout << "d1=" << distortion.at<double>(0,0) << std::endl;
  std::cout << "d2=" << distortion.at<double>(0,1) << std::endl;
  std::cout << "d3=" << distortion.at<double>(0,2) << std::endl;
  std::cout << "d4=" << distortion.at<double>(0,3) << std::endl;
  std::cout << "R1=" << rvec.at<double>(0,0) << std::endl;
  std::cout << "R2=" << rvec.at<double>(0,1) << std::endl;
  std::cout << "R3=" << rvec.at<double>(0,2) << std::endl;
  std::cout << "Tx=" << tvec.at<double>(0,0) << std::endl;
  std::cout << "Ty=" << tvec.at<double>(0,1) << std::endl;
  std::cout << "Tz=" << tvec.at<double>(0,2) << std::endl;

  double tol = 0.01;

  REQUIRE( fabs(intrinsic.at<double>(0,0) - eFx) < tol );
  REQUIRE( fabs(intrinsic.at<double>(1,1) - eFy) < tol );
  REQUIRE( fabs(intrinsic.at<double>(0,2) - eCx) < tol );
  REQUIRE( fabs(intrinsic.at<double>(1,2) - eCy) < tol );
  REQUIRE( fabs(distortion.at<double>(0,0) - dist) < tol );
  REQUIRE( fabs(distortion.at<double>(0,1) - 0) < tol );
  REQUIRE( fabs(distortion.at<double>(0,2) - 0) < tol );
  REQUIRE( fabs(distortion.at<double>(0,3) - 0) < tol );
  REQUIRE( fabs(rms - expectedRMS) < tol );
}
