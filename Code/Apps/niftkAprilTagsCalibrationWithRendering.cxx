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
#include <niftkAprilTagsPointDetector.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkPointUtilities.h>
#include <niftkMatrixUtilities.h>
#include <niftkIntensityBasedCameraCalibration.h>
#include <niftkCalibratedRenderingPipeline.h>
#include <IntensityBased/Internal/niftkRenderingBasedMonoIntrinsicCostFunction.h>
#include <IntensityBased/Internal/niftkRenderingBasedMonoExtrinsicCostFunction.h>
#include <IntensityBased/Internal/niftkRenderingBasedMonoBlurringCostFunction.h>
#include <IntensityBased/Internal/niftkRenderingBasedStereoExtrinsicCostFunction.h>
#include <IntensityBased/Internal/niftkProjectionBasedMonoIntrinsicCostFunction.h>
#include <IntensityBased/Internal/niftkProjectionBasedMonoExtrinsicCostFunction.h>
#include <IntensityBased/Internal/niftkProjectionBasedStereoExtrinsicCostFunction.h>
#include <cv.h>
#include <highgui.h>
#include <cstdlib>
#include <QApplication>
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>

double CalculateRMSOnUndistortedPoints(const niftk::Model3D& model,
                                       const std::vector<cv::Mat>& colourLeftImages,
                                       const std::vector<cv::Mat>& colourRightImages,
                                       const std::string& tagFamily,
                                       const cv::Point2d& imageScaleFactors,
                                       const cv::Mat& intrinsicLeft,
                                       const cv::Mat& distortionLeft,
                                       const std::vector<cv::Mat>& rvecsLeft,
                                       const std::vector<cv::Mat>& tvecsLeft,
                                       const cv::Mat& intrinsicRight,
                                       const cv::Mat& distortionRight,
                                       const cv::Mat& leftToRightRotationMatrix,
                                       const cv::Mat& leftToRightTranslationVector
                                       )
{
  std::vector<cv::Mat> filteredRVecLeft;
  std::vector<cv::Mat> filteredTVecLeft;

  std::list<niftk::PointSet> listOfPointsLeft;
  std::list<niftk::PointSet> listOfPointsRight;
  niftk::PointSet pointSetLeft;
  niftk::PointSet pointSetRight;

  for (int i = 0; i < colourLeftImages.size(); i++)
  {
    cv::Mat greyLeft;
    cv::cvtColor(colourLeftImages[i], greyLeft, CV_BGR2GRAY);

    cv::Mat mapX = cv::Mat::zeros(colourLeftImages[i].rows, colourLeftImages[i].cols, CV_32FC1);
    cv::Mat mapY = cv::Mat::zeros(colourLeftImages[i].rows, colourLeftImages[i].cols, CV_32FC1);
    cv::initUndistortRectifyMap(intrinsicLeft, distortionLeft, cv::noArray(), intrinsicLeft, colourLeftImages[i].size(), CV_32FC1, mapX, mapY);

    cv::Mat undistortedLeft;
    cv::remap(greyLeft, undistortedLeft, mapX, mapY, CV_INTER_LANCZOS4);

    niftk::AprilTagsPointDetector detectorLeft(false, tagFamily, 0.8, 0.8);
    detectorLeft.SetImage(&undistortedLeft);
    detectorLeft.SetImageScaleFactor(imageScaleFactors);
    pointSetLeft = detectorLeft.GetPoints();

    cv::Mat greyRight;
    cv::cvtColor(colourRightImages[i], greyRight, CV_BGR2GRAY);

    mapX = cv::Mat::zeros(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
    mapY = cv::Mat::zeros(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
    cv::initUndistortRectifyMap(intrinsicRight, distortionRight, cv::noArray(), intrinsicRight, colourRightImages[i].size(), CV_32FC1, mapX, mapY);

    cv::Mat undistortedRight;
    cv::remap(greyRight, undistortedRight, mapX, mapY, CV_INTER_LANCZOS4);

    niftk::AprilTagsPointDetector detectorRight(false, tagFamily, 0.8, 0.8);
    detectorRight.SetImage(&undistortedRight);
    detectorRight.SetImageScaleFactor(imageScaleFactors);
    pointSetRight = detectorRight.GetPoints();

    if (   pointSetLeft.size() > 3
        && pointSetRight.size() > 3
       )
    {
      listOfPointsLeft.push_back(pointSetLeft);
      listOfPointsRight.push_back(pointSetRight);
      filteredRVecLeft.push_back(rvecsLeft[i]);
      filteredTVecLeft.push_back(tvecsLeft[i]);
    }
    else
    {
      std::cerr << "Dropping frame i=" << i << std::endl;
    }
  }

  double rms = 0;
  cv::Point3d rmsPerAxis;
  cv::Mat zeroDistortionParams = cv::Mat::zeros(1, 5, CV_64FC1);

  rms = niftk::ComputeRMSReconstructionError(model,
                                             listOfPointsLeft,
                                             listOfPointsRight,
                                             intrinsicLeft,
                                             zeroDistortionParams,
                                             filteredRVecLeft,
                                             filteredTVecLeft,
                                             intrinsicRight,
                                             zeroDistortionParams,
                                             leftToRightRotationMatrix,
                                             leftToRightTranslationVector,
                                             rmsPerAxis
                                             );
  return rms;
}


/**
* \file niftkAprilTagsCalibrationWithRendering.cxx
* \brief Very experimental test harness to try and do April Tags calibration with
* the optional addition of rendering or project based calibration.
* \ingroup applications
*/
int main(int argc, char ** argv)
{
  if (argc < 10)
  {
    std::cerr << "Usage: niftkAprilTagsCalibrationWithRendering modelPoints.txt model.vtk texture.vtk method[0=normal,1=rendering,2=projection] "
              << "tagFamily rescaleX rescaleY "
              << "leftImage1.png leftImage2.png ... leftImageN.txt "
              << "rightImage1.png rightImage2.png ... rightImageN.txt "
              << std::endl;
    return EXIT_FAILURE;
  }

  try
  {
    int numberOfArgumentsBeforeImages = 8;
    int numberOfImagesPerSide = (argc-numberOfArgumentsBeforeImages)/2;

    if ((argc - numberOfArgumentsBeforeImages)%2 != 0)
    {
      std::cerr << "Expected an even number of image files for stereo calibration" << std::endl;
      return EXIT_FAILURE;
    }

    std::string modelFileName = argv[1];
    niftk::Model3D model = niftk::LoadModel3D(modelFileName);

    int method = atoi(argv[4]);
    if (method != 0 && method != 1 && method != 2)
    {
      niftkNiftyCalThrow() << "Invalid method, should be 0, 1 or 2, meaning 'normal', 'rendering' or 'projection' respectively.";
    }

    std::string tagFamily = argv[5];

    float rescaleX = atof(argv[6]);
    if (rescaleX < 0)
    {
      niftkNiftyCalThrow() << "Negative scale factors are not allowed.";
    }
    float rescaleY = atof(argv[7]);
    if (rescaleY < 0)
    {
      niftkNiftyCalThrow() << "Negative scale factors are not allowed.";
    }

    cv::Point2d scaleFactors;
    scaleFactors.x = rescaleX;
    scaleFactors.y = rescaleY;

    cv::Size2i imageSize;
    niftk::PointSet pointSet;
    std::list<niftk::PointSet> listOfPointsLeft;
    std::list<niftk::PointSet> listOfPointsRight;

    std::vector<cv::Mat> colourLeftImages;
    std::vector<cv::Mat> colourRightImages;

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

      niftk::AprilTagsPointDetector detector(false, tagFamily, 0.8, 0.8);
      detector.SetImage(&greyImage);
      detector.SetImageScaleFactor(scaleFactors);
      pointSet = detector.GetPoints();

      if (pointSet.size() > 0)
      {
        if (i-numberOfArgumentsBeforeImages < numberOfImagesPerSide)
        {
          listOfPointsLeft.push_back(pointSet);
          colourLeftImages.push_back(image);
        }
        else
        {
          listOfPointsRight.push_back(pointSet);
          colourRightImages.push_back(image);
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
    cv::Mat leftToRightTranslationVector;

    cv::Matx21d result = niftk::FullStereoCameraCalibration(model,
                                                            listOfPointsLeft,
                                                            listOfPointsRight,
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
                                                            leftToRightTranslationVector,
                                                            essentialMatrix,
                                                            fundamentalMatrix,
                                                            false // optimise 3D
                                                           );
    cv::Mat leftToRightRotationVector;
    cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);

    std::cout << "niftkAprilTagsCalibrationWithRendering:(" << imageSize.width << "," << imageSize.height <<  ") "
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
              << leftToRightTranslationVector.at<double>(1,0) << " "
              << leftToRightTranslationVector.at<double>(2,0) << " "
              << result(0, 0) << " "
              << result(1, 0)
              << std::endl;

    cv::Size2i scaledImageSize;
    scaledImageSize.width = imageSize.width * scaleFactors.x;
    scaledImageSize.height = imageSize.height * scaleFactors.y;

    double rms = 0;

    if (method == 1)
    {
      QApplication app(argc, argv);

      QVTKWidget *widget = new QVTKWidget();
      widget->show();
      widget->resize(imageSize.width, imageSize.height);

      vtkRenderWindow *window = widget->GetRenderWindow();
      window->DoubleBufferOff();
      window->GetInteractor()->Disable();

      niftk::CalibratedRenderingPipeline *p = new niftk::CalibratedRenderingPipeline(scaledImageSize,
                                                                                     imageSize,
                                                                                     argv[2],
                                                                                     argv[3],
                                                                                     true);
      p->ConnectToRenderWindow(window);

      // Dump rhs, video+rendering for all views.
      for (int i = 0; i < colourRightImages.size(); i++)
      {
        std::ostringstream videoFileName;
        videoFileName << "/tmp/matt.rhs.normal.video." << i << ".png";

        cv::Mat mapX = cv::Mat::zeros(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
        cv::Mat mapY = cv::Mat::zeros(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
        cv::initUndistortRectifyMap(intrinsicRight, distortionRight, cv::noArray(), intrinsicRight, imageSize, CV_32FC1, mapX, mapY);

        cv::Mat undistorted;
        cv::remap(colourRightImages[i], undistorted, mapX, mapY, CV_INTER_LANCZOS4);

        cv::imwrite(videoFileName.str(), undistorted);

        cv::Matx44d cameraMatrix = niftk::RodriguesToMatrix(rvecsRight[i], tvecsRight[i]);

        std::ostringstream renderedFileName;
        renderedFileName << "/tmp/matt.rhs.normal.rendering." << i << ".png";

        p->SetBackgroundColour(0, 0, 255);
        p->SetIntrinsics(intrinsicRight);
        p->SetWorldToCameraMatrix(cameraMatrix);
        p->DumpScreen(renderedFileName.str());
      }

      p->SetActivated(false);
      delete p;

      rms = CalculateRMSOnUndistortedPoints(model,
                                            colourLeftImages,
                                            colourRightImages,
                                            tagFamily,
                                            scaleFactors,
                                            intrinsicLeft,
                                            distortionLeft,
                                            rvecsLeft,
                                            tvecsLeft,
                                            intrinsicRight,
                                            distortionRight,
                                            leftToRightRotationMatrix,
                                            leftToRightTranslationVector
                                           );

      std::cout << "niftkAprilTagsCalibrationWithRendering: 3D error on undistorted points: " << rms << std::endl;

      niftk::RenderingBasedMonoBlurringCostFunction::Pointer blurLeft = niftk::RenderingBasedMonoBlurringCostFunction::New();
      blurLeft->Initialise(window,
                           scaledImageSize,
                           imageSize,
                           argv[2],
                           argv[3],
                           colourLeftImages,
                           intrinsicLeft,
                           distortionLeft,
                           rvecsLeft,
                           tvecsLeft
                           );
      blurLeft->SetUseBlurring(true);
      blurLeft->SetActivated(true);

      double sigmaLeft = 5;
      niftk::IntensityBasedBlurringCalibration(blurLeft.GetPointer(), sigmaLeft);

      blurLeft->SetActivated(false);

      niftk::RenderingBasedMonoBlurringCostFunction::Pointer blurRight = niftk::RenderingBasedMonoBlurringCostFunction::New();
      blurRight->Initialise(window,
                            scaledImageSize,
                            imageSize,
                            argv[2],
                            argv[3],
                            colourLeftImages,
                            intrinsicRight,
                            distortionRight,
                            rvecsRight,
                            tvecsRight
                            );
      blurRight->SetUseBlurring(true);
      blurRight->SetActivated(true);

      double sigmaRight = 5;
      niftk::IntensityBasedBlurringCalibration(blurRight.GetPointer(), sigmaRight);

      blurRight->SetActivated(false);

      std::cerr << "niftkAprilTagsCalibrationWithRendering: blurring revealed left=" << sigmaLeft << ", right=" << sigmaRight << std::endl;

      niftk::RenderingBasedMonoIntrinsicCostFunction::Pointer leftIntrinsicCostFunction = niftk::RenderingBasedMonoIntrinsicCostFunction::New();
      leftIntrinsicCostFunction->Initialise(window,
                                            scaledImageSize,
                                            imageSize,
                                            argv[2],
                                            argv[3],
                                            colourLeftImages,
                                            rvecsLeft,
                                            tvecsLeft
                                           );
      leftIntrinsicCostFunction->SetSigma(sigmaLeft);
      leftIntrinsicCostFunction->SetUseBlurring(true);

      niftk::RenderingBasedMonoIntrinsicCostFunction::Pointer rightIntrinsicCostFunction = niftk::RenderingBasedMonoIntrinsicCostFunction::New();
      rightIntrinsicCostFunction->Initialise(window,
                                             scaledImageSize,
                                             imageSize,
                                             argv[2],
                                             argv[3],
                                             colourRightImages,
                                             rvecsRight,
                                             tvecsRight
                                            );
      rightIntrinsicCostFunction->SetSigma(sigmaRight);
      rightIntrinsicCostFunction->SetUseBlurring(true);

      niftk::RenderingBasedStereoExtrinsicCostFunction::Pointer stereoExtrinsicCostFunction = niftk::RenderingBasedStereoExtrinsicCostFunction::New();
      stereoExtrinsicCostFunction->Initialise(window,
                                              scaledImageSize,
                                              imageSize,
                                              argv[2],
                                              argv[3],
                                              colourLeftImages,
                                              colourRightImages,
                                              intrinsicLeft,
                                              distortionLeft,
                                              intrinsicRight,
                                              distortionRight,
                                              leftToRightRotationMatrix,
                                              leftToRightTranslationVector
                                             );
      stereoExtrinsicCostFunction->SetSigma(sigmaLeft);
      stereoExtrinsicCostFunction->SetSigmaRight(sigmaRight);
      stereoExtrinsicCostFunction->SetUseBlurring(true);

      niftk::IntensityBasedStereoCameraCalibration(leftIntrinsicCostFunction.GetPointer(),
                                                   rightIntrinsicCostFunction.GetPointer(),
                                                   stereoExtrinsicCostFunction.GetPointer(),
                                                   intrinsicLeft,
                                                   distortionLeft,
                                                   rvecsLeft,
                                                   tvecsLeft,
                                                   intrinsicRight,
                                                   distortionRight,
                                                   rvecsRight,
                                                   tvecsRight,
                                                   leftToRightRotationMatrix,
                                                   leftToRightTranslationVector
                                                  );

      p = new niftk::CalibratedRenderingPipeline(scaledImageSize,
                                                 imageSize,
                                                 argv[2],
                                                 argv[3],
                                                 true);
      p->ConnectToRenderWindow(window);

      // Dump rhs, video+rendering for all views.
      for (int i = 0; i < colourRightImages.size(); i++)
      {
        std::ostringstream videoFileName;
        videoFileName << "/tmp/matt.rhs.optimised.video." << i << ".png";

        cv::Mat mapX = cv::Mat::zeros(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
        cv::Mat mapY = cv::Mat::zeros(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
        cv::initUndistortRectifyMap(intrinsicRight, distortionRight, cv::noArray(), intrinsicRight, imageSize, CV_32FC1, mapX, mapY);

        cv::Mat undistorted;
        cv::remap(colourRightImages[i], undistorted, mapX, mapY, CV_INTER_LANCZOS4);

        cv::imwrite(videoFileName.str(), undistorted);

        cv::Matx44d cameraMatrix = niftk::RodriguesToMatrix(rvecsRight[i], tvecsRight[i]);

        std::ostringstream renderedFileName;
        renderedFileName << "/tmp/matt.rhs.optimised.rendering." << i << ".png";

        p->SetBackgroundColour(0, 0, 255);
        p->SetIntrinsics(intrinsicRight);
        p->SetWorldToCameraMatrix(cameraMatrix);
        p->DumpScreen(renderedFileName.str());
      }

      p->SetActivated(false);
      delete p;

      rms = CalculateRMSOnUndistortedPoints(model,
                                            colourLeftImages,
                                            colourRightImages,
                                            tagFamily,
                                            scaleFactors,
                                            intrinsicLeft,
                                            distortionLeft,
                                            rvecsLeft,
                                            tvecsLeft,
                                            intrinsicRight,
                                            distortionRight,
                                            leftToRightRotationMatrix,
                                            leftToRightTranslationVector
                                           );

      std::cout << "niftkAprilTagsCalibrationWithRendering: 3D error after rendering: " << rms << std::endl;

    }
    else if (method == 2)
    {
      niftk::ProjectionBasedMonoIntrinsicCostFunction::Pointer leftIntrinsicCostFunction = niftk::ProjectionBasedMonoIntrinsicCostFunction::New();
      leftIntrinsicCostFunction->Initialise(imageSize,
                                            argv[2],
                                            colourLeftImages,
                                            rvecsLeft,
                                            tvecsLeft
                                            );

      niftk::ProjectionBasedMonoIntrinsicCostFunction::Pointer rightIntrinsicCostFunction = niftk::ProjectionBasedMonoIntrinsicCostFunction::New();
      rightIntrinsicCostFunction->Initialise(imageSize,
                                             argv[2],
                                             colourRightImages,
                                             rvecsRight,
                                             tvecsRight
                                             );

      niftk::ProjectionBasedStereoExtrinsicCostFunction::Pointer stereoExtrinsicCostFunction = niftk::ProjectionBasedStereoExtrinsicCostFunction::New();
      stereoExtrinsicCostFunction->Initialise(imageSize,
                                              argv[2],
                                              colourLeftImages,
                                              colourRightImages,
                                              intrinsicLeft,
                                              distortionLeft,
                                              intrinsicRight,
                                              distortionRight,
                                              leftToRightRotationMatrix,
                                              leftToRightTranslationVector
                                              );

      niftk::IntensityBasedStereoCameraCalibration(leftIntrinsicCostFunction.GetPointer(),
                                                   rightIntrinsicCostFunction.GetPointer(),
                                                   stereoExtrinsicCostFunction.GetPointer(),
                                                   intrinsicLeft,
                                                   distortionLeft,
                                                   rvecsLeft,
                                                   tvecsLeft,
                                                   intrinsicRight,
                                                   distortionRight,
                                                   rvecsRight,
                                                   tvecsRight,
                                                   leftToRightRotationMatrix,
                                                   leftToRightTranslationVector
                                                  );

      rms = CalculateRMSOnUndistortedPoints(model,
                                            colourLeftImages,
                                            colourRightImages,
                                            tagFamily,
                                            scaleFactors,
                                            intrinsicLeft,
                                            distortionLeft,
                                            rvecsLeft,
                                            tvecsLeft,
                                            intrinsicRight,
                                            distortionRight,
                                            leftToRightRotationMatrix,
                                            leftToRightTranslationVector
                                           );

      std::cout << "niftkAprilTagsCalibrationWithRendering: 3D error after projection: " << rms << std::endl;

    }

    if (method == 1 || method == 2)
    {
      std::cout << "niftkAprilTagsCalibrationWithRendering-optimised:(" << imageSize.width << "," << imageSize.height <<  ") "
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
                << leftToRightTranslationVector.at<double>(1,0) << " "
                << leftToRightTranslationVector.at<double>(2,0) << " "
                << rms << " "
                << std::endl;
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
