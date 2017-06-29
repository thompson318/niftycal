/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <niftkIOUtilities.h>
#include <niftkIterativeStereoCameraCalibration.h>
#include <niftkAprilTagsPointDetector.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkPointUtilities.h>
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

/**
* \file niftkStereoIterativeAprilTagsCalibration.cxx
* \brief Calibrate stereo camera, using AprilTag features, and
* the <a href="http://dx.doi.org/10.1109/ICCVW.2009.5457474">Datta 2009</a> iterative optimisation.
* \ingroup applications
*/
int main(int argc, char ** argv)
{
  if (argc < 11)
  {
    std::cerr << "Usage: niftkStereoIterativeAprilTagsCalibration modelPoints.txt "
              << " referenceImage.png referencePoints tagFamily rescaleX rescaleY zeroDistortion optimise3D "
              << "leftImage1.png leftImage2.png ... leftImageN.txt "
              << "rightImage1.png rightImage2.png ... rightImageN.txt "
              << std::endl;
    return EXIT_FAILURE;
  }

  try
  {
    int numberOfArgumentsBeforeImages = 9;
    int numberOfImagesPerSide = (argc-numberOfArgumentsBeforeImages)/2;

    if ((argc - numberOfArgumentsBeforeImages)%2 != 0)
    {
      std::cerr << "Expected an even number of image files" << std::endl;
      return EXIT_FAILURE;
    }

    std::string modelFileName = argv[1];
    niftk::Model3D model = niftk::LoadModel3D(modelFileName);

    std::string referenceImageFile = argv[2];
    cv::Mat referenceImage = cv::imread(referenceImageFile);
    cv::Mat referenceImageGreyScale;
    cv::cvtColor(referenceImage, referenceImageGreyScale, CV_BGR2GRAY);

    std::string referencePointsFile = argv[3];
    std::pair< cv::Mat, niftk::PointSet> referenceImageData;
    referenceImageData.first = referenceImageGreyScale;
    referenceImageData.second = niftk::LoadPointSet(referencePointsFile);

    std::string tagFamily = argv[4];
    float rescaleX = atof(argv[5]);
    if (rescaleX < 0)
    {
      niftkNiftyCalThrow() << "Negative scale factors are not allowed.";
    }
    float rescaleY = atof(argv[6]);
    if (rescaleY < 0)
    {
      niftkNiftyCalThrow() << "Negative scale factors are not allowed.";
    }

    int   zeroDistortion = atoi(argv[7]);
    int   optimise3D = atoi(argv[8]);

    cv::Point2d originalScaleFactors;
    originalScaleFactors.x = rescaleX;
    originalScaleFactors.y = rescaleY;

    cv::Point2d warpedImageScaleFactors;
    warpedImageScaleFactors.x = 1;
    warpedImageScaleFactors.y = 1;

    cv::Size2i imageSize;

    std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImagesLeft;
    std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarpingLeft;

    std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImagesRight;
    std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarpingRight;

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
      cv::Mat greyImageClone = greyImage.clone();

      niftk::AprilTagsPointDetector* detector1 = new niftk::AprilTagsPointDetector(false, // don't include corners, slightly more accurate
                                                                                   tagFamily, 0, 0);
      detector1->SetImageScaleFactor(originalScaleFactors);

      niftk::AprilTagsPointDetector* detector2 = new niftk::AprilTagsPointDetector(false, // don't include corners, slightly more accurate
                                                                                   tagFamily, 0, 0);
      detector2->SetImageScaleFactor(warpedImageScaleFactors);

      if (i-numberOfArgumentsBeforeImages < numberOfImagesPerSide)
      {
        std::shared_ptr<niftk::IPoint2DDetector> originalDetector(detector1);
        originalImagesLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
        dynamic_cast<niftk::AprilTagsPointDetector*>(originalImagesLeft.back().first.get())->SetImage(&(originalImagesLeft.back().second));

        std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(detector2);
        imagesForWarpingLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
        dynamic_cast<niftk::AprilTagsPointDetector*>(imagesForWarpingLeft.back().first.get())->SetImage(&(imagesForWarpingLeft.back().second));
      }
      else
      {
        std::shared_ptr<niftk::IPoint2DDetector> originalDetector(detector1);
        originalImagesRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
        dynamic_cast<niftk::AprilTagsPointDetector*>(originalImagesRight.back().first.get())->SetImage(&(originalImagesRight.back().second));

        std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(detector2);
        imagesForWarpingRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
        dynamic_cast<niftk::AprilTagsPointDetector*>(imagesForWarpingRight.back().first.get())->SetImage(&(imagesForWarpingRight.back().second));
      }
    }

    if (originalImagesLeft.size() == 0)
    {
      niftkNiftyCalThrow() << "No left hand camera images available.";
    }
    if (originalImagesRight.size() == 0)
    {
      niftkNiftyCalThrow() << "No right hand camera points available.";
    }
    if (originalImagesLeft.size() != originalImagesRight.size())
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

    cv::Matx21d rms = niftk::IterativeStereoCameraCalibration(
          model,
          referenceImageData,
          originalImagesLeft,
          originalImagesRight,
          imageSize,
          imagesForWarpingLeft,
          intrinsicLeft,
          distortionLeft,
          rvecsLeft,
          tvecsLeft,
          imagesForWarpingRight,
          intrinsicRight,
          distortionRight,
          rvecsRight,
          tvecsRight,
          leftToRightRotationMatrix,
          leftToRightTranslationVector,
          essentialMatrix,
          fundamentalMatrix,
          flags,
          optimise3D
          );

    cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);

    std::cout << "niftkStereoIterativeAprilTagsCalibration:(" << imageSize.width << "," << imageSize.height <<  ") "
              << originalImagesLeft.size() << " "
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
              << rms(0, 0) << " "
              << rms(1, 0)
              << std::endl;


    QApplication app(niftk::argc, niftk::argv);

    QVTKWidget *widget = new QVTKWidget();
    widget->show();
    widget->resize(imageSize.width, imageSize.height);

    vtkRenderWindow *window = widget->GetRenderWindow();
    window->DoubleBufferOff();
    window->GetInteractor()->Disable();

    niftk::CalibratedRenderingPipeline *p = new niftk::CalibratedRenderingPipeline(imageSize, imageSize,
                                                                                   "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                                                                   "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
                                                                                   true);
    p->ConnectToRenderWindow(window);

    // Dump rhs, video+rendering for all views.
    for (int i = 0; i < colourRightImages.size(); i++)
    {
      std::ostringstream videoFileName;
      videoFileName << "/tmp/matt.rhs.normal.video." << i << ".png";

      cv::Mat mapX = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
      cv::Mat mapY = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
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

    std::list<niftk::PointSet> listOfPointsOnUndistortedLeft;
    std::list<niftk::PointSet> listOfPointsOnUndistortedRight;

    for (int i = 0; i < colourLeftImages.size(); i++)
    {
      cv::Mat greyLeft;
      cv::cvtColor(colourLeftImages[i], greyLeft, CV_BGR2GRAY);

      cv::Mat mapX = cvCreateMat(colourLeftImages[i].rows, colourLeftImages[i].cols, CV_32FC1);
      cv::Mat mapY = cvCreateMat(colourLeftImages[i].rows, colourLeftImages[i].cols, CV_32FC1);
      cv::initUndistortRectifyMap(intrinsicLeft, distortionLeft, cv::noArray(), intrinsicLeft, imageSize, CV_32FC1, mapX, mapY);

      cv::Mat undistorted;
      cv::remap(greyLeft, undistorted, mapX, mapY, CV_INTER_LANCZOS4);

      niftk::ChessboardPointDetector detector(corners);
      detector.SetImage(&undistorted);
      pointSet = detector.GetPoints();
      if (pointSet.size() != numberInternalCornersInX * numberInternalCornersInY)
      {
        niftkNiftyCalThrow() << "Failed to extract left points, i=" << i;
      }
      listOfPointsOnUndistortedLeft.push_back(pointSet);
    }

    for (int i = 0; i < colourRightImages.size(); i++)
    {
      cv::Mat greyRight;
      cv::cvtColor(colourRightImages[i], greyRight, CV_BGR2GRAY);

      cv::Mat mapX = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
      cv::Mat mapY = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
      cv::initUndistortRectifyMap(intrinsicRight, distortionRight, cv::noArray(), intrinsicRight, imageSize, CV_32FC1, mapX, mapY);

      cv::Mat undistorted;
      cv::remap(greyRight, undistorted, mapX, mapY, CV_INTER_LANCZOS4);

      niftk::ChessboardPointDetector detector(corners);
      detector.SetImage(&undistorted);
      pointSet = detector.GetPoints();
      if (pointSet.size() != numberInternalCornersInX * numberInternalCornersInY)
      {
        niftkNiftyCalThrow() << "Failed to extract right points, i=" << i;
      }

      listOfPointsOnUndistortedRight.push_back(pointSet);
    }

    // Recompute RMS error
    cv::Mat zeroDistortionParams = cv::Mat::zeros(1, 5, CV_64FC1);

    cv::Point3d rmsPerAxis;
    double rmsAgain = niftk::ComputeRMSReconstructionError(model,
                                                           listOfPointsOnUndistortedLeft,
                                                           listOfPointsOnUndistortedRight,
                                                           intrinsicLeft,
                                                           zeroDistortionParams,
                                                           rvecsLeft,
                                                           tvecsLeft,
                                                           intrinsicRight,
                                                           zeroDistortionParams,
                                                           leftToRightRotationMatrix,
                                                           leftToRightTranslationVector,
                                                           rmsPerAxis
                                                           );

    cv::Mat rvec;
    cv::Rodrigues(leftToRightRotationMatrix, rvec);

    std::cout << "Stereo RMS-2D=" << result(0, 0) << std::endl;
    std::cout << "Stereo RMS-3D=" << result(1, 0) << std::endl;
    std::cout << "Stereo RMS-3D=" << rmsAgain << std::endl;
    std::cout << "Stereo RMS-3Dx=" << rmsPerAxis.x << std::endl;
    std::cout << "Stereo RMS-3Dy=" << rmsPerAxis.y << std::endl;
    std::cout << "Stereo RMS-3Dz=" << rmsPerAxis.z << std::endl;
    std::cout << "Stereo R1=" << rvec.at<double>(0,0) << std::endl;
    std::cout << "Stereo R2=" << rvec.at<double>(0,1) << std::endl;
    std::cout << "Stereo R3=" << rvec.at<double>(0,2) << std::endl;
    std::cout << "Stereo T1=" << leftToRightTranslationVector.at<double>(0,0) << std::endl;
    std::cout << "Stereo T2=" << leftToRightTranslationVector.at<double>(1,0) << std::endl;
    std::cout << "Stereo T3=" << leftToRightTranslationVector.at<double>(2,0) << std::endl;
    std::cout << "Stereo Fxl=" << intrinsicLeft.at<double>(0,0) << std::endl;
    std::cout << "Stereo Fyl=" << intrinsicLeft.at<double>(1,1) << std::endl;
    std::cout << "Stereo Cxl=" << intrinsicLeft.at<double>(0,2) << std::endl;
    std::cout << "Stereo Cyl=" << intrinsicLeft.at<double>(1,2) << std::endl;
    std::cout << "Stereo K1l=" << distortionLeft.at<double>(0,0) << std::endl;
    std::cout << "Stereo K2l=" << distortionLeft.at<double>(0,1) << std::endl;
    std::cout << "Stereo P1l=" << distortionLeft.at<double>(0,2) << std::endl;
    std::cout << "Stereo P2l=" << distortionLeft.at<double>(0,3) << std::endl;
    std::cout << "Stereo Fxr=" << intrinsicRight.at<double>(0,0) << std::endl;
    std::cout << "Stereo Fyr=" << intrinsicRight.at<double>(1,1) << std::endl;
    std::cout << "Stereo Cxr=" << intrinsicRight.at<double>(0,2) << std::endl;
    std::cout << "Stereo Cyr=" << intrinsicRight.at<double>(1,2) << std::endl;
    std::cout << "Stereo K1r=" << distortionRight.at<double>(0,0) << std::endl;
    std::cout << "Stereo K2r=" << distortionRight.at<double>(0,1) << std::endl;
    std::cout << "Stereo P1r=" << distortionRight.at<double>(0,2) << std::endl;
    std::cout << "Stereo P2r=" << distortionRight.at<double>(0,3) << std::endl;

    niftk::RenderingBasedMonoBlurringCostFunction::Pointer blurLeft = niftk::RenderingBasedMonoBlurringCostFunction::New();
    blurLeft->Initialise(window,
                         imageSize,
                         imageSize,
                         "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                         "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
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
                          imageSize,
                          imageSize,
                          "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                          "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
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

    std::cerr << "Matt, blurring revealed left=" << sigmaLeft << ", right=" << sigmaRight << std::endl;

    niftk::RenderingBasedMonoIntrinsicCostFunction::Pointer leftIntrinsicCostFunction = niftk::RenderingBasedMonoIntrinsicCostFunction::New();
    leftIntrinsicCostFunction->Initialise(window,
                                          imageSize,
                                          imageSize,
                                          "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                          "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
                                          colourLeftImages,
                                          rvecsLeft,
                                          tvecsLeft
                                         );
    leftIntrinsicCostFunction->SetSigma(sigmaLeft);
    leftIntrinsicCostFunction->SetUseBlurring(true);

    niftk::RenderingBasedMonoIntrinsicCostFunction::Pointer rightIntrinsicCostFunction = niftk::RenderingBasedMonoIntrinsicCostFunction::New();
    rightIntrinsicCostFunction->Initialise(window,
                                           imageSize,
                                           imageSize,
                                           "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                           "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
                                           colourRightImages,
                                           rvecsRight,
                                           tvecsRight
                                          );
    rightIntrinsicCostFunction->SetSigma(sigmaRight);
    rightIntrinsicCostFunction->SetUseBlurring(true);

    niftk::RenderingBasedMonoExtrinsicCostFunction::Pointer leftExtrinsicCostFunction = niftk::RenderingBasedMonoExtrinsicCostFunction::New();
    leftExtrinsicCostFunction->Initialise(window, imageSize, imageSize,
                                          "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                          "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
                                          colourLeftImages,
                                          intrinsicLeft,
                                          distortionLeft
                                         );
    leftExtrinsicCostFunction->SetSigma(sigmaLeft);
    leftExtrinsicCostFunction->SetUseBlurring(true);

    niftk::RenderingBasedStereoExtrinsicCostFunction::Pointer stereoExtrinsicCostFunction = niftk::RenderingBasedStereoExtrinsicCostFunction::New();
    stereoExtrinsicCostFunction->Initialise(window,
                                            imageSize,
                                            imageSize,
                                            "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                            "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
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

  /*
    niftk::ProjectionBasedMonoIntrinsicCostFunction::Pointer leftIntrinsicCostFunction = niftk::ProjectionBasedMonoIntrinsicCostFunction::New();
    leftIntrinsicCostFunction->Initialise(imageSize,
                                          "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                          colourLeftImages,
                                          rvecsLeft,
                                          tvecsLeft
                                          );

    niftk::ProjectionBasedMonoIntrinsicCostFunction::Pointer rightIntrinsicCostFunction = niftk::ProjectionBasedMonoIntrinsicCostFunction::New();
    rightIntrinsicCostFunction->Initialise(imageSize,
                                           "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                           colourRightImages,
                                           rvecsRight,
                                           tvecsRight
                                           );

    niftk::ProjectionBasedStereoExtrinsicCostFunction::Pointer stereoExtrinsicCostFunction = niftk::ProjectionBasedStereoExtrinsicCostFunction::New();
    stereoExtrinsicCostFunction->Initialise(imageSize,
                                            "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                            colourLeftImages,
                                            colourRightImages,
                                            intrinsicLeft,
                                            distortionLeft,
                                            intrinsicRight,
                                            distortionRight,
                                            leftToRightRotationMatrix,
                                            leftToRightTranslationVector
                                            );

    */

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

    p = new niftk::CalibratedRenderingPipeline(imageSize, imageSize,
                                               "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3.vtk",
                                               "/Users/mattclarkson/build/NiftyCal/Testing/Data/VTK/chess-14x10x3-large.png",
                                               true);
    p->ConnectToRenderWindow(window);

    // Dump rhs, video+rendering for all views.
    for (int i = 0; i < colourRightImages.size(); i++)
    {
      std::ostringstream videoFileName;
      videoFileName << "/tmp/matt.rhs.optimised.video." << i << ".png";

      cv::Mat mapX = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
      cv::Mat mapY = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
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

    std::vector<cv::Mat> optimisedRVecLeft;
    std::vector<cv::Mat> optimisedTVecLeft;

    std::list<niftk::PointSet> listOfPointsOnUndistortedOptimisedLeft;
    std::list<niftk::PointSet> listOfPointsOnUndistortedOptimisedRight;
    niftk::PointSet pointSetLeft;
    niftk::PointSet pointSetRight;

    for (int i = 0; i < colourLeftImages.size(); i++)
    {
      cv::Mat greyLeft;
      cv::cvtColor(colourLeftImages[i], greyLeft, CV_BGR2GRAY);

      cv::Mat mapX = cvCreateMat(colourLeftImages[i].rows, colourLeftImages[i].cols, CV_32FC1);
      cv::Mat mapY = cvCreateMat(colourLeftImages[i].rows, colourLeftImages[i].cols, CV_32FC1);
      cv::initUndistortRectifyMap(intrinsicLeft, distortionLeft, cv::noArray(), intrinsicLeft, imageSize, CV_32FC1, mapX, mapY);

      cv::Mat undistortedLeft;
      cv::remap(greyLeft, undistortedLeft, mapX, mapY, CV_INTER_LANCZOS4);

      niftk::ChessboardPointDetector leftDetector(corners);
      leftDetector.SetImage(&undistortedLeft);
      pointSetLeft = leftDetector.GetPoints();

      cv::Mat greyRight;
      cv::cvtColor(colourRightImages[i], greyRight, CV_BGR2GRAY);

      mapX = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
      mapY = cvCreateMat(colourRightImages[i].rows, colourRightImages[i].cols, CV_32FC1);
      cv::initUndistortRectifyMap(intrinsicRight, distortionRight, cv::noArray(), intrinsicRight, imageSize, CV_32FC1, mapX, mapY);

      cv::Mat undistortedRight;
      cv::remap(greyRight, undistortedRight, mapX, mapY, CV_INTER_LANCZOS4);

      niftk::ChessboardPointDetector detectorRight(corners);
      detectorRight.SetImage(&undistortedRight);
      pointSetRight = detectorRight.GetPoints();

      if (   pointSetLeft.size() == numberInternalCornersInX * numberInternalCornersInY
          && pointSetRight.size() == numberInternalCornersInX * numberInternalCornersInY
          )
      {
        listOfPointsOnUndistortedOptimisedLeft.push_back(pointSetLeft);
        listOfPointsOnUndistortedOptimisedRight.push_back(pointSetRight);
        optimisedRVecLeft.push_back(rvecsLeft[i]);
        optimisedTVecLeft.push_back(tvecsLeft[i]);
      }
      else
      {
        std::cerr << "Dropping frame i=" << i << std::endl;
      }
    }

    // Recompute RMS error
    rmsAgain = niftk::ComputeRMSReconstructionError(model,
                                                    listOfPointsOnUndistortedOptimisedLeft,
                                                    listOfPointsOnUndistortedOptimisedRight,
                                                    intrinsicLeft,
                                                    zeroDistortionParams,
                                                    optimisedRVecLeft,
                                                    optimisedTVecLeft,
                                                    intrinsicRight,
                                                    zeroDistortionParams,
                                                    leftToRightRotationMatrix,
                                                    leftToRightTranslationVector,
                                                    rmsPerAxis
                                                    );

    cv::Rodrigues(leftToRightRotationMatrix, rvec);

    std::cout << "Stereo RMS-2D=" << result(0, 0) << std::endl;
    std::cout << "Stereo RMS-3D=" << result(1, 0) << std::endl;
    std::cout << "Stereo RMS-3D=" << rmsAgain << std::endl;
    std::cout << "Stereo RMS-3Dx=" << rmsPerAxis.x << std::endl;
    std::cout << "Stereo RMS-3Dy=" << rmsPerAxis.y << std::endl;
    std::cout << "Stereo RMS-3Dz=" << rmsPerAxis.z << std::endl;
    std::cout << "Stereo R1=" << rvec.at<double>(0,0) << std::endl;
    std::cout << "Stereo R2=" << rvec.at<double>(0,1) << std::endl;
    std::cout << "Stereo R3=" << rvec.at<double>(0,2) << std::endl;
    std::cout << "Stereo T1=" << leftToRightTranslationVector.at<double>(0,0) << std::endl;
    std::cout << "Stereo T2=" << leftToRightTranslationVector.at<double>(1,0) << std::endl;
    std::cout << "Stereo T3=" << leftToRightTranslationVector.at<double>(2,0) << std::endl;
    std::cout << "Stereo Fxl=" << intrinsicLeft.at<double>(0,0) << std::endl;
    std::cout << "Stereo Fyl=" << intrinsicLeft.at<double>(1,1) << std::endl;
    std::cout << "Stereo Cxl=" << intrinsicLeft.at<double>(0,2) << std::endl;
    std::cout << "Stereo Cyl=" << intrinsicLeft.at<double>(1,2) << std::endl;
    std::cout << "Stereo K1l=" << distortionLeft.at<double>(0,0) << std::endl;
    std::cout << "Stereo K2l=" << distortionLeft.at<double>(0,1) << std::endl;
    std::cout << "Stereo P1l=" << distortionLeft.at<double>(0,2) << std::endl;
    std::cout << "Stereo P2l=" << distortionLeft.at<double>(0,3) << std::endl;
    std::cout << "Stereo Fxr=" << intrinsicRight.at<double>(0,0) << std::endl;
    std::cout << "Stereo Fyr=" << intrinsicRight.at<double>(1,1) << std::endl;
    std::cout << "Stereo Cxr=" << intrinsicRight.at<double>(0,2) << std::endl;
    std::cout << "Stereo Cyr=" << intrinsicRight.at<double>(1,2) << std::endl;
    std::cout << "Stereo K1r=" << distortionRight.at<double>(0,0) << std::endl;
    std::cout << "Stereo K2r=" << distortionRight.at<double>(0,1) << std::endl;
    std::cout << "Stereo P1r=" << distortionRight.at<double>(0,2) << std::endl;
    std::cout << "Stereo P2r=" << distortionRight.at<double>(0,3) << std::endl;

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
