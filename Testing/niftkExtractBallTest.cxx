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
#include <niftkBallDetector.h>
#include <niftkWhiteBallDetector.h>
#include <niftkRedBallDetector.h>
#include <niftkIOUtilities.h>
#include <niftkPointUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Extract ball", "[ball]" ) {

  if (niftk::argc != 9 && niftk::argc != 10)
  {
    std::cerr << "Usage: niftkExtractBallTest image scaleX scaleY expectedImageWidth expectedImageHeight method expectedX expectedY [outputFile]" << std::endl;
    REQUIRE( niftk::argc >= 9);
    REQUIRE( niftk::argc <= 10);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  int scaleX = atoi(niftk::argv[2]);
  int scaleY = atoi(niftk::argv[3]);
  int expectedWidth = atoi(niftk::argv[4]);
  int expectedHeight = atoi(niftk::argv[5]);
  int method = atoi(niftk::argv[6]);
  double expectedX = atof(niftk::argv[7]);
  double expectedY = atof(niftk::argv[8]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Point2d scaleFactors;
  scaleFactors.x = scaleX;
  scaleFactors.y = scaleY;

  std::unique_ptr<niftk::BallDetector> detector;
  if (method == 0)
  {
    detector.reset(new niftk::RedBallDetector());
  }
  else
  {
    detector.reset(new niftk::WhiteBallDetector());
  }
  detector->SetImage(&image);
  detector->SetImageScaleFactor(scaleFactors);

  niftk::PointSet points = detector->GetPoints();

  niftk::DumpPoints(std::cout, points);

  REQUIRE( points.size() == 1 );
  REQUIRE( fabs(points[0].point.x - expectedX) < 0.1);
  REQUIRE( fabs(points[0].point.y - expectedY) < 0.1);

  if (niftk::argc == 10 && points.size() > 0)
  {
    std::string outputFile = niftk::argv[9];
    niftk::SavePointSet(points, outputFile);
  }


}
