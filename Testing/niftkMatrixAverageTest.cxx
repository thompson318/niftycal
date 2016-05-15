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
#include <niftkMatrixUtilities.h>
#include <niftkIOUtilities.h>
#include <cv.h>

TEST_CASE( "Average matrices", "[matrices]" ) {

  int expectedNumberOfArguments =  4;
  if (niftk::argc < expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkMatrixAverageTest averageMatrix m1 m2 m3 ... mN" << std::endl;
    REQUIRE( niftk::argc >= expectedNumberOfArguments);
  }

  cv::Mat tmp = niftk::LoadMatrix(niftk::argv[1]);
  cv::Matx44d expectedAverage(tmp);

  std::list<cv::Matx44d > matrices;

  for (int i = 2; i < niftk::argc; i++)
  {
    cv::Mat tmp = niftk::LoadMatrix(niftk::argv[i]);
    cv::Matx44d tmp2(tmp);
    matrices.push_back(tmp2);
  }

  cv::Matx44d actualAverage = niftk::AverageMatricesUsingEigenValues(matrices);

  std::cerr << "Expected=" << std::endl << expectedAverage << std::endl;
  std::cerr << "Actual=" << std::endl << actualAverage << std::endl;

  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 4; c++)
    {
      REQUIRE(fabs(actualAverage(r,c) - expectedAverage(r,c)) < 0.01);
    }
  }
}
