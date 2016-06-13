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
#include <niftkPointUtilities.h>
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalException.h>
#include <cv.h>
#include <list>
#include <cstdlib>
#include <random>

/**
 * \brief Runs stereo simulation, using pre-extracted points, to
 * assess the 3D reconstruction error, and the effect of noise.
 */
int main(int argc, char ** argv)
{
  if (argc < 10)
  {
    std::cerr << "Usage: niftkStereoSimulationFromPoints sigma iters imageSizeX imageSizeY modelPoints.txt "
              << "leftImagePoints1.txt leftImagePoints2.txt ... leftImagePointsN.txt "
              << "rightImagePoints1.txt rightImagePoints2.txt ... rightImagePointsN.txt " << std::endl;

    return EXIT_FAILURE;
  }

  try
  {
    int numberOfArgumentsBeforeImages = 6;
    int numberOfImagesPerSide = (argc-numberOfArgumentsBeforeImages)/2;

    if ((argc - numberOfArgumentsBeforeImages)%2 != 0)
    {
      std::cerr << "Expected an even number of point files" << std::endl;
      return EXIT_FAILURE;
    }

    float sigma = atof(argv[1]);
    int iterations = atoi(argv[2]);
    int sizeX = atoi(argv[3]);
    int sizeY = atoi(argv[4]);
    std::string modelFile = argv[5];

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
    cv::Mat leftToRightRotationMatrix;
    cv::Mat leftToRightRotationVector;
    cv::Mat leftToRightTranslation;

    // First we do a 'standard' stereo calibration.
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
                                                leftToRightRotationMatrix,
                                                leftToRightTranslation,
                                                essentialMatrix,
                                                fundamentalMatrix,
                                                CV_CALIB_USE_INTRINSIC_GUESS
                                               );

    cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);
    cv::Matx14d leftToRightAxisAngle = niftk::RodriguesToAxisAngle(leftToRightRotationVector);

    std::cout << "niftkStereoCalibrationFromPoints(initial):(" << imageSize.width << "," << imageSize.height <<  ") "
              << leftPoints.size() << " "
              << intrinsicLeft.at<double>(0,0) << " "
              << intrinsicLeft.at<double>(1,1) << " "
              << intrinsicLeft.at<double>(0,2) << " "
              << intrinsicLeft.at<double>(1,2) << " "
              << distortionLeft.at<double>(0,0) << " "
              << distortionLeft.at<double>(0,1) << " "
              << distortionLeft.at<double>(0,2) << " "
              << distortionLeft.at<double>(0,3) << " "
              << distortionLeft.at<double>(0,4) << " "
              << intrinsicRight.at<double>(0,0) << " "
              << intrinsicRight.at<double>(1,1) << " "
              << intrinsicRight.at<double>(0,2) << " "
              << intrinsicRight.at<double>(1,2) << " "
              << distortionRight.at<double>(0,0) << " "
              << distortionRight.at<double>(0,1) << " "
              << distortionRight.at<double>(0,2) << " "
              << distortionRight.at<double>(0,3) << " "
              << distortionRight.at<double>(0,4) << " "
              << leftToRightRotationVector.at<double>(0,0) << " "
              << leftToRightRotationVector.at<double>(0,1) << " "
              << leftToRightRotationVector.at<double>(0,2) << " "
              << leftToRightTranslation.at<double>(0,0) << " "
              << leftToRightTranslation.at<double>(0,1) << " "
              << leftToRightTranslation.at<double>(0,2) << " "
              << leftToRightAxisAngle(0, 0) << " "
              << leftToRightAxisAngle(0, 1) << " "
              << leftToRightAxisAngle(0, 2) << " "
              << leftToRightAxisAngle(0, 3) << " "
              << rms
              << std::endl;

    // Now we do simulation. First project all 3D points to 2D points,
    // to create a pseudo gold-standard.
    std::list<niftk::PointSet> leftGoldStandardPoints;
    std::list<niftk::PointSet>::const_iterator leftIter;
    int viewCounter = 0;
    for (leftIter = leftPoints.begin(); leftIter != leftPoints.end(); ++leftIter)
    {
      std::vector<cv::Point2f> observed;
      std::vector<cv::Point2f> projected;
      std::vector<niftk::NiftyCalIdType> ids;

      cv::Matx44d cameraMatrix = niftk::RodriguesToMatrix(rvecsLeft[viewCounter], tvecsLeft[viewCounter]);

      niftk::ProjectMatchingPoints(model,
                                   *leftIter,
                                   cameraMatrix,
                                   intrinsicLeft,
                                   distortionLeft,
                                   observed,
                                   projected,
                                   ids
                                  );
      niftk::PointSet projectedPoints;
      niftk::ConvertPoints(projected, ids, projectedPoints);
      leftGoldStandardPoints.push_back(projectedPoints);
      viewCounter++;
    }
    std::list<niftk::PointSet> rightGoldStandardPoints;
    std::list<niftk::PointSet>::const_iterator rightIter;
    viewCounter = 0;
    for (rightIter = rightPoints.begin(); rightIter != rightPoints.end(); ++rightIter)
    {
      std::vector<cv::Point2f> observed;
      std::vector<cv::Point2f> projected;
      std::vector<niftk::NiftyCalIdType> ids;

      cv::Matx44d cameraMatrix = niftk::RodriguesToMatrix(rvecsRight[viewCounter], tvecsRight[viewCounter]);

      niftk::ProjectMatchingPoints(model,
                                   *rightIter,
                                   cameraMatrix,
                                   intrinsicRight,
                                   distortionRight,
                                   observed,
                                   projected,
                                   ids
                                  );
      niftk::PointSet projectedPoints;
      niftk::ConvertPoints(projected, ids, projectedPoints);
      rightGoldStandardPoints.push_back(projectedPoints);
      viewCounter++;
    }

    // sanity check. Do calibration with pseudo gold-standard.
    niftk::MonoCameraCalibration(model,
                                 leftGoldStandardPoints,
                                 imageSize,
                                 intrinsicLeft,
                                 distortionLeft,
                                 rvecsLeft,
                                 tvecsLeft
                                );

    niftk::MonoCameraCalibration(model,
                                 rightGoldStandardPoints,
                                 imageSize,
                                 intrinsicRight,
                                 distortionRight,
                                 rvecsRight,
                                 tvecsRight
                                );

    rms = niftk::StereoCameraCalibration(model,
                                         leftGoldStandardPoints,
                                         rightGoldStandardPoints,
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
                                         CV_CALIB_USE_INTRINSIC_GUESS
                                         );

    cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);
    leftToRightAxisAngle = niftk::RodriguesToAxisAngle(leftToRightRotationVector);

    std::cout << "niftkStereoCalibrationFromPoints(gold):(" << imageSize.width << "," << imageSize.height <<  ") "
              << leftPoints.size() << " "
              << intrinsicLeft.at<double>(0,0) << " "
              << intrinsicLeft.at<double>(1,1) << " "
              << intrinsicLeft.at<double>(0,2) << " "
              << intrinsicLeft.at<double>(1,2) << " "
              << distortionLeft.at<double>(0,0) << " "
              << distortionLeft.at<double>(0,1) << " "
              << distortionLeft.at<double>(0,2) << " "
              << distortionLeft.at<double>(0,3) << " "
              << distortionLeft.at<double>(0,4) << " "
              << intrinsicRight.at<double>(0,0) << " "
              << intrinsicRight.at<double>(1,1) << " "
              << intrinsicRight.at<double>(0,2) << " "
              << intrinsicRight.at<double>(1,2) << " "
              << distortionRight.at<double>(0,0) << " "
              << distortionRight.at<double>(0,1) << " "
              << distortionRight.at<double>(0,2) << " "
              << distortionRight.at<double>(0,3) << " "
              << distortionRight.at<double>(0,4) << " "
              << leftToRightRotationVector.at<double>(0,0) << " "
              << leftToRightRotationVector.at<double>(0,1) << " "
              << leftToRightRotationVector.at<double>(0,2) << " "
              << leftToRightTranslation.at<double>(0,0) << " "
              << leftToRightTranslation.at<double>(0,1) << " "
              << leftToRightTranslation.at<double>(0,2) << " "
              << leftToRightAxisAngle(0, 0) << " "
              << leftToRightAxisAngle(0, 1) << " "
              << leftToRightAxisAngle(0, 2) << " "
              << leftToRightAxisAngle(0, 3) << " "
              << rms
              << std::endl;

    std::default_random_engine engine;
    std::normal_distribution<double> normalDistribution(0, sigma);

    for (int i = 0; i < iterations; i++)
    {
      // Now add noise to points.
      std::list<niftk::PointSet> leftNoisyPoints;
      for (leftIter = leftGoldStandardPoints.begin(); leftIter != leftGoldStandardPoints.end(); ++leftIter)
      {
        niftk::PointSet noisyPoints = niftk::AddGaussianNoise(engine, normalDistribution, *leftIter);
        leftNoisyPoints.push_back(noisyPoints);
      }
      std::list<niftk::PointSet> rightNoisyPoints;
      for (rightIter = rightGoldStandardPoints.begin(); rightIter != rightGoldStandardPoints.end(); ++rightIter)
      {
        niftk::PointSet noisyPoints = niftk::AddGaussianNoise(engine, normalDistribution, *rightIter);
        rightNoisyPoints.push_back(noisyPoints);
      }

      // Now re-run calibration.
      niftk::MonoCameraCalibration(model,
                                   leftNoisyPoints,
                                   imageSize,
                                   intrinsicLeft,
                                   distortionLeft,
                                   rvecsLeft,
                                   tvecsLeft
                                  );

      niftk::MonoCameraCalibration(model,
                                   rightNoisyPoints,
                                   imageSize,
                                   intrinsicRight,
                                   distortionRight,
                                   rvecsRight,
                                   tvecsRight
                                  );

      rms = niftk::StereoCameraCalibration(model,
                                           leftNoisyPoints,
                                           rightNoisyPoints,
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
                                           CV_CALIB_USE_INTRINSIC_GUESS
                                           );

      cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);
      leftToRightAxisAngle = niftk::RodriguesToAxisAngle(leftToRightRotationVector);

      std::cout << "niftkStereoCalibrationFromPoints(noisy):(" << imageSize.width << "," << imageSize.height <<  ") "
                << leftPoints.size() << " "
                << intrinsicLeft.at<double>(0,0) << " "
                << intrinsicLeft.at<double>(1,1) << " "
                << intrinsicLeft.at<double>(0,2) << " "
                << intrinsicLeft.at<double>(1,2) << " "
                << distortionLeft.at<double>(0,0) << " "
                << distortionLeft.at<double>(0,1) << " "
                << distortionLeft.at<double>(0,2) << " "
                << distortionLeft.at<double>(0,3) << " "
                << distortionLeft.at<double>(0,4) << " "
                << intrinsicRight.at<double>(0,0) << " "
                << intrinsicRight.at<double>(1,1) << " "
                << intrinsicRight.at<double>(0,2) << " "
                << intrinsicRight.at<double>(1,2) << " "
                << distortionRight.at<double>(0,0) << " "
                << distortionRight.at<double>(0,1) << " "
                << distortionRight.at<double>(0,2) << " "
                << distortionRight.at<double>(0,3) << " "
                << distortionRight.at<double>(0,4) << " "
                << leftToRightRotationVector.at<double>(0,0) << " "
                << leftToRightRotationVector.at<double>(0,1) << " "
                << leftToRightRotationVector.at<double>(0,2) << " "
                << leftToRightTranslation.at<double>(0,0) << " "
                << leftToRightTranslation.at<double>(0,1) << " "
                << leftToRightTranslation.at<double>(0,2) << " "
                << leftToRightAxisAngle(0, 0) << " "
                << leftToRightAxisAngle(0, 1) << " "
                << leftToRightAxisAngle(0, 2) << " "
                << leftToRightAxisAngle(0, 3) << " "
                << rms
                << std::endl;

      // Evaluate RMS
      cv::Point3d rmsInEachAxis;
      rms = niftk::ComputeRMSReconstructionError(model,
                                                 leftNoisyPoints,
                                                 rightNoisyPoints,
                                                 intrinsicLeft,
                                                 distortionLeft,
                                                 rvecsLeft,
                                                 tvecsLeft,
                                                 intrinsicRight,
                                                 distortionRight,
                                                 leftToRightRotationMatrix,
                                                 leftToRightTranslation,
                                                 rmsInEachAxis
                                                );

      std::cout << "niftkStereoSimulationFromPoints:(" << imageSize.width << "," << imageSize.height <<  ") "
                << leftPoints.size() << " "
                << rms << " "
                << rmsInEachAxis
                << std::endl;

    } // end for each iter
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
