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
#include <niftkIOUtilities.h>
#include <niftkPointUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkHandEyeCalibration.h>
#include <cv.h>

TEST_CASE( "Elvis Hand Eye", "[HandEye]" ) {

  if (niftk::argc < 22)
  {
    std::cerr << "Usage: niftkElvisHandEye intrinsic.txt 3DPoints.txt 2DPoints.txt rx ry rz tx ty tz mat_1.txt mat_2.txt ... mat_n.txt" << std::endl;
    REQUIRE( niftk::argc >= 22);
  }

  cv::Mat intrinsic = niftk::LoadMatrix(niftk::argv[1]);
  niftk::Model3D pointsIn3D = niftk::LoadModel3D(niftk::argv[2]);
  niftk::PointSet pointsIn2D = niftk::LoadPointSet(niftk::argv[3]);
  float rx = atof(niftk::argv[4]);
  float ry = atof(niftk::argv[5]);
  float rz = atof(niftk::argv[6]);
  float tx = atof(niftk::argv[7]);
  float ty = atof(niftk::argv[8]);
  float tz = atof(niftk::argv[9]);

  std::vector<cv::Matx44d > matrices;

  for (int i = 10; i < niftk::argc; i++)
  {
    cv::Mat tmp = niftk::LoadMatrix(niftk::argv[i]);
    cv::Matx44d tmp2(tmp);
    matrices.push_back(tmp2);
  }

  REQUIRE(pointsIn3D.size() == 1);
  REQUIRE(pointsIn2D.size() == matrices.size());
  REQUIRE(pointsIn2D.size() >= 12);

  niftk::Point3D point3D = pointsIn3D[0];

  std::vector<cv::Point3d> points3D;
  std::vector<cv::Point2d> points2D;

  cv::Matx44d handEye;

  for (int i = 0; i < matrices.size(); i++)
  {
    if (pointsIn2D.find(i) == pointsIn2D.end())
    {
      niftkNiftyCalThrow() << "Failed to find 2D point:" << i;
    }

    cv::Point2d p2 = pointsIn2D.find(i)->second.point;
    points2D.push_back(p2);

    cv::Point3d p3 = point3D.point;
    points3D.push_back(p3);
  }

  niftk::CalculateHandEyeUsingPoint2Line(intrinsic, matrices, points3D, points2D, 0.00001, handEye);

  std::cerr << "Hand-eye:" << std::endl << handEye << std::endl;
  std::cerr << "Hand-eye Inv:" << std::endl << handEye.inv() << std::endl;

  cv::Mat rotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat translationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(handEye, rotationVector, translationVector);

  std::cerr << "R=" << rotationVector << std::endl;
  std::cerr << "t=" << translationVector << std::endl;

  double tolerance = 0.001;
  REQUIRE(fabs(rotationVector.at<double>(0, 0) - rx) < tolerance);
  REQUIRE(fabs(rotationVector.at<double>(0, 1) - ry) < tolerance);
  REQUIRE(fabs(rotationVector.at<double>(0, 2) - rz) < tolerance);
  REQUIRE(fabs(translationVector.at<double>(0, 0) - tx) < tolerance);
  REQUIRE(fabs(translationVector.at<double>(0, 1) - ty) < tolerance);
  REQUIRE(fabs(translationVector.at<double>(0, 2) - tz) < tolerance);
}
