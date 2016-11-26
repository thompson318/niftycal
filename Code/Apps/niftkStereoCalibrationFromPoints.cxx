/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <niftkIOUtilities.h>
#include <niftkZhangCameraCalibration.h>
#include <niftkTsaiCameraCalibration.h>
#include <niftkPointUtilities.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <cv.h>
#include <list>
#include <cstdlib>

/**
* \file niftkStereoCalibrationFromPoints.cxx
* \brief Calibrate stereo camera from pre-extracted points.
*/
int main(int argc, char ** argv)
{
  if (argc < 6)
  {
    std::cerr << "Usage: niftkStereoCalibrationFromPoints imageSizeX imageSizeY modelPoints.txt "
              << "leftImagePoints1.txt leftImagePoints2.txt ... leftImagePointsN.txt "
              << "rightImagePoints1.txt rightImagePoints2.txt ... rightImagePointsN.txt " << std::endl;

    return EXIT_FAILURE;
  }

  try
  {
    int numberOfArgumentsBeforeImages = 4;
    int numberOfImagesPerSide = (argc-numberOfArgumentsBeforeImages)/2;

    if ((argc - numberOfArgumentsBeforeImages)%2 != 0)
    {
      std::cerr << "Expected an even number of point files" << std::endl;
      return EXIT_FAILURE;
    }

    int sizeX = atoi(argv[1]);
    if (sizeX < 1)
    {
      niftkNiftyCalThrow() << "Invalid imageSizeX which should be >= 1";
    }
    int sizeY = atoi(argv[2]);
    if (sizeY < 1)
    {
      niftkNiftyCalThrow() << "Invalid sizeY which should be >= 1";
    }
    cv::Size2i imageSize(sizeX, sizeY);

    std::string modelFile = argv[3];
    niftk::Model3D model = niftk::LoadModel3D(modelFile);

    std::list<niftk::PointSet> leftPoints;
    std::list<niftk::PointSet> rightPoints;

    for (int i = numberOfArgumentsBeforeImages; i < argc; i++)
    {
      niftk::PointSet p = niftk::LoadPointSet(argv[i]);
      if (p.size() >= 4) // Deep within OpenCV lies a check for at least 4 points.
      {
        if (i-numberOfArgumentsBeforeImages < numberOfImagesPerSide)
        {
          leftPoints.push_back(p);
        }
        else
        {
          rightPoints.push_back(p);
        }
      }
    }
    if (leftPoints.size() == 0)
    {
      niftkNiftyCalThrow() << "No valid left camera points were read.";
    }
    if (rightPoints.size() == 0)
    {
      niftkNiftyCalThrow() << "No valid right camera points were read.";
    }
    if (leftPoints.size() != rightPoints.size())
    {
      niftkNiftyCalThrow() << "A different number of left and right point sets were read.";
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
    cv::Mat leftToRightRotationVector;
    cv::Mat leftToRightTranslation;

    cv::Matx21d result;

    if (leftPoints.size() == 1 && rightPoints.size() == 1)
    {
      // Can try Tsai 1987 calibration.
      cv::Mat rvecLeft;
      cv::Mat tvecLeft;
      cv::Mat rvecRight;
      cv::Mat tvecRight;

      double sensorScaleInX = 1;

      cv::Point2d sensorDimensions;
      sensorDimensions.x = 1;
      sensorDimensions.y = 1;

      niftk::TsaiMonoCameraCalibration(model,
                                       *(leftPoints.begin()),
                                       imageSize,
                                       sensorDimensions,
                                       imageSize.width,
                                       sensorScaleInX,
                                       intrinsicLeft,
                                       distortionLeft,
                                       rvecLeft,
                                       tvecLeft,
                                       true // full optimisation.
                                      );

      niftk::TsaiMonoCameraCalibration(model,
                                       *(rightPoints.begin()),
                                       imageSize,
                                       sensorDimensions,
                                       imageSize.width,
                                       sensorScaleInX,
                                       intrinsicRight,
                                       distortionRight,
                                       rvecRight,
                                       tvecRight,
                                       true // full optimisation.
                                      );

      result = niftk::TsaiStereoCameraCalibration(model,
                                                  *(leftPoints.begin()),
                                                  *(rightPoints.begin()),
                                                  imageSize,
                                                  intrinsicLeft,
                                                  distortionLeft,
                                                  rvecLeft,
                                                  tvecLeft,
                                                  intrinsicRight,
                                                  distortionRight,
                                                  rvecRight,
                                                  tvecRight,
                                                  leftToRightRotationMatrix,
                                                  leftToRightTranslation,
                                                  essentialMatrix,
                                                  fundamentalMatrix,
                                                  CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_INTRINSIC,
                                                  false // optimise3D, could be command line arg.
                                                 );
    }
    else
    {
      // Can try Zhang 2000 calibration.

      niftk::ZhangMonoCameraCalibration(model,
                                        leftPoints,
                                        imageSize,
                                        intrinsicLeft,
                                        distortionLeft,
                                        rvecsLeft,
                                        tvecsLeft
                                       );

      niftk::ZhangMonoCameraCalibration(model,
                                        rightPoints,
                                        imageSize,
                                        intrinsicRight,
                                        distortionRight,
                                        rvecsRight,
                                        tvecsRight
                                       );

      result = niftk::ZhangStereoCameraCalibration(model,
                                                   leftPoints,
                                                   rightPoints,
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
                                                   leftToRightTranslation,
                                                   essentialMatrix,
                                                   fundamentalMatrix,
                                                   CV_CALIB_USE_INTRINSIC_GUESS,
                                                   false // optimise3D, could be command line arg.
                                                  );
    }

    cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);

    std::cout << "niftkStereoCalibrationFromPoints:(" << imageSize.width << "," << imageSize.height <<  ") "
              << leftPoints.size() << " "
              << intrinsicLeft.at<double>(0,0) << " "
              << intrinsicLeft.at<double>(1,1) << " "
              << intrinsicLeft.at<double>(0,2) << " "
              << intrinsicLeft.at<double>(1,2) << " ";

    for (int i = 0; i < distortionLeft.cols; i++)
    {
      std::cout << distortionLeft.at<double>(0,i) << " ";
    }

    std::cout << intrinsicRight.at<double>(0,0) << " "
              << intrinsicRight.at<double>(1,1) << " "
              << intrinsicRight.at<double>(0,2) << " "
              << intrinsicRight.at<double>(1,2) << " ";

    for (int i = 0; i < distortionRight.cols; i++)
    {
      std::cout << distortionRight.at<double>(0,i) << " ";
    }

    std::cout << leftToRightRotationVector.at<double>(0,0) << " "
              << leftToRightRotationVector.at<double>(0,1) << " "
              << leftToRightRotationVector.at<double>(0,2) << " "
              << leftToRightTranslation.at<double>(0,0) << " "
              << leftToRightTranslation.at<double>(0,1) << " "
              << leftToRightTranslation.at<double>(0,2) << " "
              << result(0, 0) << " " // 2D error
              << result(1, 0)        // 3D error
              << std::endl;
  }
  catch (niftk::NiftyCalException& e)
  {
    std::cerr << "Caught exception:" << e.GetDescription() << std::endl
              << "              in:" << e.GetFileName() << std::endl
              << "         at line:" << e.GetLineNumber()
              << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
