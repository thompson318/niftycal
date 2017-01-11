/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <niftkIOUtilities.h>
#include <niftkStereoCameraCalibration.h>
#include <niftkPointUtilities.h>
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <cv.h>
#include <list>
#include <cstdlib>

/**
 * \file niftkStereoSimulationByParameter.cxx
 * \brief Runs stereo simulation, using pre-extracted points, to
 * assess the 3D reconstruction error with calibration parameter changes.
 * \ingroup applications
 */
int main(int argc, char ** argv)
{
  if (argc < 6)
  {
    std::cerr << "Usage: niftkStereoSimulationByParameter imageSizeX imageSizeY modelPoints.txt "
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

    result = niftk::FullStereoCameraCalibration(model,
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
                                                false // could be command line arg.
                                               );

    cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);
    cv::Matx14d leftToRightAxisAngle = niftk::RodriguesToAxisAngle(leftToRightRotationVector);

    std::cout << "niftkStereoCalibrationFromPoints(initial) " << imageSize.width << " " << imageSize.height <<  " "
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
              << leftToRightTranslation.at<double>(1,0) << " "
              << leftToRightTranslation.at<double>(2,0) << " "
              << leftToRightAxisAngle(0, 0) << " "
              << leftToRightAxisAngle(0, 1) << " "
              << leftToRightAxisAngle(0, 2) << " "
              << leftToRightAxisAngle(0, 3) << " "
              << result(0, 0) << " "
              << result(1, 0)
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

    // Sanity check. Do calibration with pseudo gold-standard, RMS should be zero.

    result = niftk::StereoCameraCalibration(model,
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
                                            false // could be command line arg.
                                           );

    cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);
    leftToRightAxisAngle = niftk::RodriguesToAxisAngle(leftToRightRotationVector);

    std::cout << "niftkStereoCalibrationFromPoints(gold) " << imageSize.width << " " << imageSize.height <<  " "
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
              << result(0, 0) << " "
              << result(1, 0)
              << std::endl;

    // Now we simply vary each parameter +/- 10%, in 100 steps, and measure reconstruction error.

    cv::Matx14d axisAngle = niftk::RodriguesToAxisAngle(leftToRightRotationVector);

    const int numberOfParameters = 4; // 3 translation, and 1 angle about axis.
    const int numberOfSteps = 101;

    double parameters[numberOfParameters] = {leftToRightTranslation.at<double>(0, 0),
                                             leftToRightTranslation.at<double>(1, 0),
                                             leftToRightTranslation.at<double>(2, 0),
                                             axisAngle(0, 3) // angle about axis (presumably nearly vertical).
                                            };

    double pi = 3.141592653589793;
    for (int p = 0; p < numberOfParameters; p++)
    {
      double originalValue = parameters[p];
      double minValue = 0;
      double maxValue = 0;
      double stepSize = 0;

      if (p < 3)
      {
        // Translation
        minValue = parameters[p] - 1; // mm
        maxValue = parameters[p] + 1; // mm
        stepSize = (maxValue - minValue)/static_cast<double>(numberOfSteps - 1);
      }
      else
      {
        // Rotation
        minValue = parameters[p] - (1*pi/180);
        maxValue = parameters[p] + (1*pi/180);
        stepSize = (maxValue - minValue)/static_cast<double>(numberOfSteps - 1);
      }

      parameters[p] = minValue;

      for (int s = 0; s < numberOfSteps; s++)
      {
        cv::Mat tmpLeftToRightTrans = cvCreateMat(3, 1, CV_64FC1);
        tmpLeftToRightTrans.at<double>(0, 0) = parameters[0];
        tmpLeftToRightTrans.at<double>(1, 0) = parameters[1];
        tmpLeftToRightTrans.at<double>(2, 0) = parameters[2];

        cv::Matx14d tmpAxisAngle = axisAngle;
        tmpAxisAngle(0, 3) = parameters[3];

        cv::Mat tmpLeftToRightRotVec = niftk::AxisAngleToRodrigues(tmpAxisAngle);
        cv::Mat tmpLeftToRightRotMat = cvCreateMat(3, 3, CV_64FC1);
        cv::Rodrigues(tmpLeftToRightRotVec, tmpLeftToRightRotMat);

        cv::Point3d rmsPerAxis;
        double rms = niftk::ComputeRMSReconstructionError(model,
                                                          leftGoldStandardPoints,
                                                          rightGoldStandardPoints,
                                                          intrinsicLeft,
                                                          distortionLeft,
                                                          rvecsLeft,
                                                          tvecsLeft,
                                                          intrinsicRight,
                                                          distortionRight,
                                                          tmpLeftToRightRotMat,
                                                          tmpLeftToRightTrans,
                                                          rmsPerAxis
                                                         );

        std::cout << "niftkStereoSimulationByParameter "
                  << p << " "
                  << s << " "
                  << parameters[0] << " "
                  << parameters[1] << " "
                  << parameters[2] << " "
                  << parameters[3] << " "
                  << 180*parameters[3]/pi << " "
                  << rmsPerAxis.x << " "
                  << rmsPerAxis.y << " "
                  << rmsPerAxis.z << " "
                  << rms
                  << std::endl;

        parameters[p] += stepSize;
      }
      parameters[p] = originalValue;
    }
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
