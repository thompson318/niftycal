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

TEST_CASE( "Matrix IO", "[matrices]" ) {

  int expectedNumberOfArguments =  3;
  if (niftk::argc < expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkMatrixIOTest input.4x4 output.4x4" << std::endl;
    REQUIRE( niftk::argc >= expectedNumberOfArguments);
  }

  cv::Mat tmp = niftk::LoadMatrix(niftk::argv[1]);
  REQUIRE(tmp.rows == 4);
  REQUIRE(tmp.cols == 4);

  cv::Matx44d inputMatrix(tmp);

  cv::Mat rotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat expectedRotationMatrix = cvCreateMat(3, 3, CV_64FC1);
  cv::Mat expectedTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat expectedTranslationVectorTransposed = cvCreateMat(3, 1, CV_64FC1);

  niftk::MatrixToRodrigues(inputMatrix, rotationVector, expectedTranslationVector);
  cv::Rodrigues(rotationVector, expectedRotationMatrix);
  expectedTranslationVectorTransposed = expectedTranslationVector.t();

  niftk::SaveNifTKStereoExtrinsics(expectedRotationMatrix, expectedTranslationVectorTransposed, niftk::argv[2]);

  cv::Mat actualRotationMatrix = cvCreateMat(3, 3, CV_64FC1);
  cv::Mat actualTranslationVector = cvCreateMat(3, 1, CV_64FC1);
  niftk::LoadNifTKStereoExtrinsics(niftk::argv[2], actualRotationMatrix, actualTranslationVector);

  cv::Matx44d actualMatrix = niftk::RotationAndTranslationToMatrix(actualRotationMatrix, actualTranslationVector);

  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 4; c++)
    {
      REQUIRE(fabs(actualMatrix(r,c) - inputMatrix(r,c)) < 0.0001);
    }
  }

}
