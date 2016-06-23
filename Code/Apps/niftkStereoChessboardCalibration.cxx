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
#include <niftkNonLinearStereoCalibrationOptimiser.h>
#include <niftkChessboardPointDetector.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkPointUtilities.h>
#include <cv.h>
#include <highgui.h>
#include <cstdlib>

/**
* \file niftkStereoChessboardCalibration.cxx
* \brief Calibrate stereo camera, using standard chessboard and OpenCV.
*/
int main(int argc, char ** argv)
{
  if (argc < 13)
  {
    std::cerr << "Usage: niftkStereChessboardCalibration modelFileName cornersInX cornersInY rescaleX rescaleY zeroDistortion "
              << "leftImage1.png leftImage2.png ... leftImageN.txt"
              << "rightImage1.png rightImage2.png ... rightImageN.txt"
              << std::endl;
    return EXIT_FAILURE;
  }

  try
  {
    int numberOfArgumentsBeforeImages = 7;
    int numberOfImagesPerSide = (argc-numberOfArgumentsBeforeImages)/2;

    if ((argc - numberOfArgumentsBeforeImages)%2 != 0)
    {
      std::cerr << "Expected an even number of image files" << std::endl;
      return EXIT_FAILURE;
    }

    std::string modelFileName = argv[1];
    niftk::Model3D model = niftk::LoadModel3D(modelFileName);

    int numberInternalCornersInX = atoi(argv[2]);
    if (numberInternalCornersInX < 2)
    {
      niftkNiftyCalThrow() << "numberInternalCornersInX < 2.";
    }

    int numberInternalCornersInY = atoi(argv[3]);
    if (numberInternalCornersInY < 2)
    {
      niftkNiftyCalThrow() << "numberInternalCornersInY < 2.";
    }
    cv::Size2i corners(numberInternalCornersInX, numberInternalCornersInY);

    float rescaleX = atof(argv[4]);
    if (rescaleX < 0)
    {
      niftkNiftyCalThrow() << "Negative scale factors are not allowed.";
    }
    float rescaleY = atof(argv[5]);
    if (rescaleY < 0)
    {
      niftkNiftyCalThrow() << "Negative scale factors are not allowed.";
    }

    cv::Point2d scaleFactors;
    scaleFactors.x = rescaleX;
    scaleFactors.y = rescaleY;

    int   zeroDistortion = atoi(argv[6]);

    cv::Size2i imageSize;
    niftk::PointSet pointSet;
    std::list<niftk::PointSet> listOfPointsLeft;
    std::list<niftk::PointSet> listOfPointsRight;

    for (int i = numberOfArgumentsBeforeImages; i < argc; i++)
    {
      cv::Mat image = cv::imread(argv[i]);
      if (i == numberOfArgumentsBeforeImages)
      {
        imageSize.width = image.cols;
        imageSize.height = image.rows;
      }
      else
      {
        if (   image.cols != imageSize.width
            || image.rows != imageSize.height
            )
        {
          niftkNiftyCalThrow() << "Invalid image size:" << image.cols << "x" << image.rows << std::endl;
        }
      }

      cv::Mat greyImage;
      cv::cvtColor(image, greyImage, CV_BGR2GRAY);

      niftk::ChessboardPointDetector detector(corners);
      detector.SetImage(&greyImage);
      detector.SetImageScaleFactor(scaleFactors);
      pointSet = detector.GetPoints();

      if (pointSet.size() > 0)
      {
        if (i-numberOfArgumentsBeforeImages < numberOfImagesPerSide)
        {
          listOfPointsLeft.push_back(pointSet);
        }
        else
        {
          listOfPointsRight.push_back(pointSet);
        }
      }
    }

    if (listOfPointsLeft.size() == 0)
    {
      niftkNiftyCalThrow() << "No left hand camera points available.";
    }
    if (listOfPointsRight.size() == 0)
    {
      niftkNiftyCalThrow() << "No right hand camera points available.";
    }
    if (listOfPointsLeft.size() != listOfPointsRight.size())
    {
      niftkNiftyCalThrow() << "Different number of views for left and right camera.";
    }

    int flags = 0;
    if (zeroDistortion == 1)
    {
      flags = cv::CALIB_ZERO_TANGENT_DIST
          | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2
          | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4
          | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
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
    cv::Mat leftToRightTranslationVector;

    niftk::MonoCameraCalibration(model,
                                 listOfPointsLeft,
                                 imageSize,
                                 intrinsicLeft,
                                 distortionLeft,
                                 rvecsLeft,
                                 tvecsLeft,
                                 flags
                                );

    niftk::MonoCameraCalibration(model,
                                 listOfPointsRight,
                                 imageSize,
                                 intrinsicRight,
                                 distortionRight,
                                 rvecsRight,
                                 tvecsRight,
                                 flags
                                );

    double rms = niftk::StereoCameraCalibration(model,
                                                listOfPointsLeft,
                                                listOfPointsRight,
                                                imageSize,
                                                intrinsicLeft,
                                                distortionLeft,
                                                intrinsicRight,
                                                distortionRight,
                                                leftToRightRotationMatrix,
                                                leftToRightTranslationVector,
                                                essentialMatrix,
                                                fundamentalMatrix,
                                                flags | CV_CALIB_USE_INTRINSIC_GUESS
                                               );

    niftk::ComputeStereoExtrinsics(model,
                                   listOfPointsLeft,
                                   imageSize,
                                   intrinsicLeft,
                                   distortionLeft,
                                   leftToRightRotationMatrix,
                                   leftToRightTranslationVector,
                                   rvecsLeft,
                                   tvecsLeft,
                                   rvecsRight,
                                   tvecsRight
                                  );

    cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);

    // Evaluate RMS reconstruction error.
    cv::Point3d rmsInEachAxis;
    double rmsReconstructionError = niftk::ComputeRMSReconstructionError(model,
                                                                         listOfPointsLeft,
                                                                         listOfPointsRight,
                                                                         intrinsicLeft,
                                                                         distortionLeft,
                                                                         rvecsLeft,
                                                                         tvecsLeft,
                                                                         intrinsicRight,
                                                                         distortionRight,
                                                                         leftToRightRotationMatrix,
                                                                         leftToRightTranslationVector,
                                                                         rmsInEachAxis
                                                                        );

    niftk::NonLinearStereoCalibrationOptimiser::Pointer optimiser =
        niftk::NonLinearStereoCalibrationOptimiser::New();
    optimiser->SetModelAndPoints(&model, &listOfPointsLeft, &listOfPointsRight);

    double optimisedRMS = optimiser->Optimise(intrinsicLeft,
                                              distortionLeft,
                                              intrinsicRight,
                                              distortionRight,
                                              rvecsLeft,
                                              tvecsLeft,
                                              leftToRightRotationMatrix,
                                              leftToRightTranslationVector
                                             );

    std::cout << "niftkStereoChessboardCalibration:(" << imageSize.width << "," << imageSize.height <<  ") "
              << listOfPointsLeft.size() << " "
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
              << leftToRightTranslationVector.at<double>(0,0) << " "
              << leftToRightTranslationVector.at<double>(0,1) << " "
              << leftToRightTranslationVector.at<double>(0,2) << " "
              << rmsInEachAxis.x << " "
              << rmsInEachAxis.y << " "
              << rmsInEachAxis.z << " "
              << rms << " "
              << rmsReconstructionError << " "
              << optimisedRMS << " "
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
