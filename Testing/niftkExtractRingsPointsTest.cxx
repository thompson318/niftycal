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
#include <niftkRingsPointDetector.h>
#include <niftkNiftyCalExceptionMacro.h>

#include <cv.h>
#include <highgui.h>
#include <iostream>

TEST_CASE( "Extract symetric rings points", "[rings]" ) {

  if (niftk::argc != 12 && niftk::argc != 13 && niftk::argc != 14)
  {
    std::cerr << "Usage: niftkExtractRingsPointsTest image referenceImage referencePoints templateImage expectedImageWidth expectedImageHeight expectedColumns expectedCirclesPerColumn maxArea method tolerance [expectedPoints] [outputFile]" << std::endl;
    REQUIRE( niftk::argc >= 12);
    REQUIRE( niftk::argc <= 14);
  }

  cv::Mat image = cv::imread(niftk::argv[1]);
  cv::Mat referenceImage = cv::imread(niftk::argv[2]);
  std::string referencePointsFileName = niftk::argv[3];
  cv::Mat templateImage = cv::imread(niftk::argv[4]);
  int expectedWidth = atoi(niftk::argv[5]);
  int expectedHeight = atoi(niftk::argv[6]);
  int ringsInX = atoi(niftk::argv[7]);
  int ringsInY = atoi(niftk::argv[8]);
  unsigned long int maxArea = atoi(niftk::argv[9]);
  int method = atoi(niftk::argv[10]);
  double tolerance = atof(niftk::argv[11]);

  REQUIRE( image.cols == expectedWidth );
  REQUIRE( image.rows == expectedHeight );

  cv::Mat greyImage;
  cv::cvtColor(image, greyImage, CV_BGR2GRAY);

  cv::Mat greyReference;
  cv::cvtColor(referenceImage, greyReference, CV_BGR2GRAY);

  cv::Mat greyTemplate;
  cv::cvtColor(templateImage, greyTemplate, CV_BGR2GRAY);

  niftk::PointSet referencePoints = niftk::LoadPointSet(referencePointsFileName);
  REQUIRE( referencePoints.size() == ringsInX * ringsInY );

  cv::Size2i patternSize(ringsInY, ringsInX);
  cv::Size2i offsetSize(10, 10);

  niftk::RingsPointDetector detector(patternSize, offsetSize);
  detector.SetImage(&greyImage);
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

  niftk::PointSet points = detector.GetPoints();
  REQUIRE( points.size() == ringsInX * ringsInY );

  if (niftk::argc >= 13 && points.size() > 0)
  {
    std::string expectedPointsFileName = niftk::argv[12];

    if (expectedPointsFileName != "dummy")
    {
      niftk::PointSet expectedPoints = niftk::LoadPointSet(expectedPointsFileName);
      REQUIRE( expectedPoints.size() == ringsInX * ringsInY );

      // check expected points
      niftk::PointSet::const_iterator iter;
      niftk::PointSet::const_iterator actualIter;

      for (iter = expectedPoints.begin();
           iter != expectedPoints.end();
           ++iter
           )
      {
        niftk::Point2D exp = (*iter).second;
        actualIter = points.find((*iter).first);

        if (actualIter == points.end())
        {
          niftkNiftyCalThrow() << "Failed to find point:" << (*iter).first;
        }

        niftk::Point2D actual = (*actualIter).second;
        REQUIRE(fabs((*iter).second.point.x - (*actualIter).second.point.x) < tolerance);
        REQUIRE(fabs((*iter).second.point.y - (*actualIter).second.point.y) < tolerance);
      }
    }
  }

  if (niftk::argc >= 14 && points.size() > 0)
  {
    std::string outputFile = niftk::argv[13];
    niftk::SavePointSet(points, outputFile);
  }
}
