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

  cv::Matx44d handEye;
  std::vector<std::pair<cv::Point2d, cv::Point3d> > pairedPoints;

  for (int i = 0; i < matrices.size(); i++)
  {
    if (pointsIn2D.find(i) == pointsIn2D.end())
    {
      niftkNiftyCalThrow() << "Failed to find 2D point:" << i;
    }

    cv::Point2d p2 = pointsIn2D.find(i)->second.point;

    // Need 3D point, in coordinate system of laparoscope marker.
    cv::Matx44d tmp = matrices[i].inv();
    cv::Matx41d pointInTrackerSpace;
    pointInTrackerSpace(0, 0) = point3D.point.x;
    pointInTrackerSpace(1, 0) = point3D.point.y;
    pointInTrackerSpace(2, 0) = point3D.point.z;
    pointInTrackerSpace(3, 0) = 1;

    cv::Matx41d pointInMarkerSpace = tmp * pointInTrackerSpace;

    cv::Point3d p3;
    p3.x = pointInMarkerSpace(0, 0);
    p3.y = pointInMarkerSpace(1, 0);
    p3.z = pointInMarkerSpace(2, 0);

    pairedPoints.push_back(std::pair<cv::Point2d, cv::Point3d>(p2, p3));
  }

  niftk::CalculateHandEyeUsingPoint2Line(intrinsic, pairedPoints, 0.00001, handEye);
  std::cerr << "Hand-eye:" << std::endl << handEye << std::endl;

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
