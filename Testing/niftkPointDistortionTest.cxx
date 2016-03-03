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
#include <niftkPointUtilities.h>
#include <niftkIOUtilities.h>

#include <cv.h>

TEST_CASE( "Check distortion/undistortion", "[points]" ) {

  int expectedNumberOfArguments =  7;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkPointDistortionTest intrinsic.txt distortion.txt sizeX sizeY px py" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  std::string intrinsicFileName = niftk::argv[1];
  std::string distortionFileName = niftk::argv[2];
  int sizeX = atoi(niftk::argv[3]);
  int sizeY = atoi(niftk::argv[4]);
  float px = atof(niftk::argv[5]);
  float py = atof(niftk::argv[6]);

  cv::Mat intrinsic = niftk::LoadMatrix(intrinsicFileName);
  cv::Mat distortion = niftk::LoadMatrix(distortionFileName);

  niftk::PointSet distorted; // in image from camera - they start off distorted.
  niftk::Point2D p;
  p.id = 0;
  p.point.x = px;
  p.point.y = py;
  distorted.insert(niftk::IdPoint2D(p.id, p));
  REQUIRE(distorted.size() == 1);

  niftk::PointSet undistorted;
  niftk::PointSet redistorted;
  niftk::UndistortPoints(distorted, intrinsic, distortion, undistorted);
  niftk::DistortPoints(undistorted, intrinsic, distortion, redistorted);
  REQUIRE(undistorted.size() == 1);
  REQUIRE(redistorted.size() == 1);
  REQUIRE((*(distorted.find(0))).first == (*(redistorted.find(0))).first);
  REQUIRE((*(distorted.find(0))).second.point == (*(redistorted.find(0))).second.point);
}
