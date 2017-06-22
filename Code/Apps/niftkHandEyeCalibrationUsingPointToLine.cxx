/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <niftkIOUtilities.h>
#include <niftkTimingCalibration.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkWhiteBallDetector.h>
#include <niftkRedBallDetector.h>
#include <niftkHandEyeCalibration.h>
#include <highgui.h>

/**
* \file niftkHandEyeCalibrationUsingPointToLine.cxx
* \brief Calibrates using method from Isabella Morgan / Elvis Chen's IPCAI 2017 paper.
* \ingroup applications
*/
int main(int argc, char ** argv)
{
  if (argc < 13
      || argc % 2 == 0) // should be odd number args.
  {
    std::cerr << "Usage: niftkHandEyeCalibrationUsingPointToLine camera3x3Matrix.txt cameraDistortion1x4Matrix.txt [white=0|red=1] "
              << " ballX ballY ballZ "
              << " videoImage1.png videoImage2.png ... videoImageN.png"
              << " trackingMatrix1.txt trackingMatrix2.txt ... trackingMatrixN.txt" << std::endl;
    return EXIT_FAILURE;
  }

  try
  {
    cv::Mat cameraMatrix = niftk::LoadMatrix(argv[1]);
    cv::Mat distortionMatrix = niftk::LoadMatrix(argv[2]);
    int method = atoi(argv[3]);
    float ballX = atof(argv[4]);
    float ballY = atof(argv[5]);
    float ballZ = atof(argv[6]);
    cv::Point3d ballPositionInTrackerSpace;
    ballPositionInTrackerSpace.x = ballX;
    ballPositionInTrackerSpace.y = ballY;
    ballPositionInTrackerSpace.z = ballZ;

    int numberOfPreliminaryArgs = 7;
    int numberOfDataArgs = argc - numberOfPreliminaryArgs;
    int numberOfSamples = numberOfDataArgs / 2;

    std::vector<cv::Matx44d> trackingMatrices;
    std::vector<cv::Point3d> pointsInTrackerSpace;
    std::vector<cv::Point2d> undistortedPoints;

    for (int i = 0; i < numberOfSamples; i++)
    {
      cv::Mat imageInColour = cv::imread(argv[numberOfPreliminaryArgs + i]);
      if (method == 1)
      {
        cv::Mat undistortedImage;
        cv::undistort(imageInColour, undistortedImage, cameraMatrix, distortionMatrix);

        niftk::RedBallDetector detector;
        detector.SetImage(&undistortedImage);

        niftk::PointSet p = detector.GetPoints();
        if (p.size() != 1)
        {
          niftkNiftyCalThrow() << "Did not find 1 point in:" << argv[numberOfPreliminaryArgs + i] << ", actually found:" << p.size();
        }

        cv::Mat tmp = niftk::LoadMatrix(argv[numberOfPreliminaryArgs + i + numberOfSamples]);
        cv::Matx44d trackingMatrix(tmp);

        trackingMatrices.push_back(trackingMatrix);
        pointsInTrackerSpace.push_back(ballPositionInTrackerSpace);
        undistortedPoints.push_back((*(p.begin())).second.point);
      }
    }

    cv::Matx44d handEye;
    niftk::CalculateHandEyeUsingPoint2Line(cameraMatrix, trackingMatrices, pointsInTrackerSpace, undistortedPoints, 0.0001, handEye);

    std::cout << "niftkHandEyeCalibrationUsingPointToLine: " << std::endl
              << handEye
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
