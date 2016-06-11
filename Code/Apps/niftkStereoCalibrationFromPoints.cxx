/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <niftkIOUtilities.h>
#include <niftkMonoCameraCalibration.h>
#include <niftkStereoCameraCalibration.h>
#include <cv.h>
#include <list>
#include <cstdlib>

/**
 * \brief Calibrate stereo camera, using pre-extracted points,
 * using niftk::StereoCameraCalibration routine.
 */
int main(int argc, char ** argv)
{
  if (argc < 8)
  {
    std::cerr << "Usage: niftkStereoCalibrationFromPoints imageSizeX imageSizeY modelPoints.txt "
              << "leftImagePoints1.txt leftImagePoints2.txt ... leftImagePointsN.txt "
              << "rightImagePoints1.txt rightImagePoints2.txt ... rightImagePointsN.txt " << std::endl;

    return EXIT_FAILURE;
  }

  int numberOfArgumentsBeforeImages = 4;
  int numberOfImagesPerSide = (argc-numberOfArgumentsBeforeImages)/2;

  if ((argc - numberOfArgumentsBeforeImages)%2 != 0)
  {
    std::cerr << "Expected an even number of point files" << std::endl;
    return EXIT_FAILURE;
  }

  int sizeX = atoi(argv[1]);
  int sizeY = atoi(argv[2]);
  std::string modelFile = argv[3];

  cv::Size2i imageSize(sizeX, sizeY);
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
  cv::Mat leftToRightRotation;
  cv::Mat leftToRightTranslation;

  niftk::MonoCameraCalibration(model,
                               leftPoints,
                               imageSize,
                               intrinsicLeft,
                               distortionLeft,
                               rvecsLeft,
                               tvecsLeft
                              );

  niftk::MonoCameraCalibration(model,
                               rightPoints,
                               imageSize,
                               intrinsicRight,
                               distortionRight,
                               rvecsRight,
                               tvecsRight
                              );

  double rms = niftk::StereoCameraCalibration(model,
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
                                              leftToRightRotation,
                                              leftToRightTranslation,
                                              essentialMatrix,
                                              fundamentalMatrix,
                                              CV_CALIB_USE_INTRINSIC_GUESS
                                             );

  std::cout << "niftkStereoCalibrationFromPoints:(" << imageSize.width << "," << imageSize.height <<  ") "
            << leftPoints.size() << " "
            << intrinsicLeft.at<double>(0,0) << " "
            << intrinsicLeft.at<double>(1,1) << " "
            << intrinsicLeft.at<double>(0,2) << " "
            << intrinsicLeft.at<double>(1,2) << " "
            << distortionLeft.at<double>(0,0) << " "
            << distortionLeft.at<double>(0,1) << " "
            << distortionLeft.at<double>(0,2) << " "
            << distortionLeft.at<double>(0,3) << " "
            << intrinsicRight.at<double>(0,0) << " "
            << intrinsicRight.at<double>(1,1) << " "
            << intrinsicRight.at<double>(0,2) << " "
            << intrinsicRight.at<double>(1,2) << " "
            << distortionRight.at<double>(0,0) << " "
            << distortionRight.at<double>(0,1) << " "
            << distortionRight.at<double>(0,2) << " "
            << distortionRight.at<double>(0,3) << " "
            << leftToRightRotation.at<double>(0,0) << " "
            << leftToRightRotation.at<double>(0,1) << " "
            << leftToRightRotation.at<double>(0,2) << " "
            << leftToRightTranslation.at<double>(0,0) << " "
            << leftToRightTranslation.at<double>(0,1) << " "
            << leftToRightTranslation.at<double>(0,2) << " "
            << rms
            << std::endl;

  return EXIT_SUCCESS;
}
