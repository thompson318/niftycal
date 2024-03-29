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
#include <niftkTemplateRingsPointDetector.h>
#include <niftkNiftyCalExceptionMacro.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Extract symetric rings points", "[rings]" ) {

  if (niftk::argc != 16 && niftk::argc != 17 && niftk::argc != 18)
  {
    std::cerr << "Usage: niftkExtractRingsPointsTest image referenceImage referencePoints templateImage scaleX scaleY expectedImageWidth expectedImageHeight expectedColumns expectedCirclesPerColumn expectedNumberPoints maxArea method asymmetric tolerance [expectedPoints] [outputFile]" << std::endl;
    REQUIRE( niftk::argc >= 16);
    REQUIRE( niftk::argc <= 18);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  cv::Mat referenceImage = cv::imread(niftk::argv[2]);
  std::string referencePointsFileName = niftk::argv[3];
  cv::Mat templateImage = cv::imread(niftk::argv[4]);
  int scaleX = atoi(niftk::argv[5]);
  int scaleY = atoi(niftk::argv[6]);
  int expectedWidth = atoi(niftk::argv[7]);
  int expectedHeight = atoi(niftk::argv[8]);
  int ringsInX = atoi(niftk::argv[9]);
  int ringsInY = atoi(niftk::argv[10]);
  int expectedNumberOfRings = atoi(niftk::argv[11]);
  unsigned long int maxArea = atoi(niftk::argv[12]);
  int method = atoi(niftk::argv[13]);
  int asymmetric = atoi(niftk::argv[14]);
  double tolerance = atof(niftk::argv[15]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Point2d scaleFactors;
  scaleFactors.x = scaleX;
  scaleFactors.y = scaleY;

  cv::Mat greyImage;
  cv::cvtColor(image, greyImage, CV_BGR2GRAY);

  cv::Mat greyReference;
  cv::cvtColor(referenceImage, greyReference, CV_BGR2GRAY);

  cv::Mat greyTemplate;
  cv::cvtColor(templateImage, greyTemplate, CV_BGR2GRAY);

  niftk::PointSet referencePoints = niftk::LoadPointSet(referencePointsFileName);
  REQUIRE( referencePoints.size() == ringsInX * ringsInY );

  cv::Size2i patternSize(ringsInX, ringsInY);
  cv::Size2i offsetSize(10, 10);

  int flags = cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING;
  if (asymmetric == 1)
  {
    flags = cv::CALIB_CB_ASYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING;
  }

  niftk::TemplateRingsPointDetector detector(patternSize, offsetSize, flags);
  detector.SetImage(&greyImage);
  detector.SetImageScaleFactor(scaleFactors);
  detector.SetTemplateImage(&greyTemplate);
  detector.SetReferenceImage(&greyReference);
  detector.SetReferencePoints(referencePoints);
  detector.SetMaxAreaInPixels(maxArea);

  if (method == 0)
  {
    detector.SetUseContours(true);
    detector.SetUseInternalResampling(false);
    detector.SetUseTemplateMatching(false);
  }
  else
  {
    detector.SetUseContours(true);
    detector.SetUseInternalResampling(true);
    detector.SetUseTemplateMatching(true);
  }

  niftk::PointSet points;

  if (expectedNumberOfRings == 0)
  {
    REQUIRE_NOTHROW(points = detector.GetPoints());
  }
  else
  {
    points = detector.GetPoints();
    REQUIRE( points.size() == expectedNumberOfRings );
  }

  if (niftk::argc >= 17 && points.size() > 0)
  {
    std::string expectedPointsFileName = niftk::argv[16];

    niftk::DumpPoints(std::cerr, points);
    if (expectedPointsFileName != "dummy")
    {
      niftk::PointSet expectedPoints = niftk::LoadPointSet(expectedPointsFileName);
      REQUIRE( expectedPoints.size() == ringsInX * ringsInY );
      REQUIRE(niftk::MatchesToWithinTolerance(points, expectedPoints, tolerance));
    }
  }

  if (niftk::argc >= 18 && points.size() > 0)
  {
    std::string outputFile = niftk::argv[17];
    niftk::SavePointSet(points, outputFile);
  }
}
