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
#include <niftkRingsPointDetector.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <cv.h>
#include <highgui.h>
#include <cstdlib>

/**
* \file niftkStereoIterativeRingsCalibration.cxx
* \brief Calibrate stereo camera, using Ring features, and
* the <a href="http://dx.doi.org/10.1109/ICCVW.2009.5457474">Datta 2009</a> iterative optimisation.
*/
int main(int argc, char ** argv)
{
  if (argc < 17)
  {
    std::cerr << "Usage: niftkStereoIterativeRingsCalibration modelPoints.txt "
              << " referenceImage.png referencePoints templateImage dotsInX dotsInY rescaleX rescaleY zeroDistortion optimise3D "
              << "leftImage1.png leftImage2.png ... leftImageN.txt "
              << "rightImage1.png rightImage2.png ... rightImageN.txt "
              << std::endl;
    return EXIT_FAILURE;
  }

  try
  {
    int numberOfArgumentsBeforeImages = 11;
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

    std::string templateImageFile = argv[4];
    cv::Mat templateImage = cv::imread(templateImageFile);
    cv::Mat templateImageGreyScale;
    cv::cvtColor(templateImage, templateImageGreyScale, CV_BGR2GRAY);

    int dotsInX = atoi(argv[5]);
    if (dotsInX < 2)
    {
      niftkNiftyCalThrow() << "dotsInX < 2.";
    }
    int dotsInY = atoi(argv[6]);
    if (dotsInY < 2)
    {
      niftkNiftyCalThrow() << "dotsInY < 2.";
    }

    float rescaleX = atof(argv[7]);
    if (rescaleX < 0)
    {
      niftkNiftyCalThrow() << "Negative scale factors are not allowed.";
    }

    float rescaleY = atof(argv[8]);
    if (rescaleY < 0)
    {
      niftkNiftyCalThrow() << "Negative scale factors are not allowed.";
    }

    int   zeroDistortion = atoi(argv[9]);
    int   optimise3D = atoi(argv[10]);

    // This is passed through to cv::findCirclesGrid, for which the documentation
    // says "patternSize = Size(points_per_row, points_per_colum)"
    // which is the opposite way round to the normal cv::Size2i constructor.
    cv::Size2i dots;
    dots.width = dotsInY;
    dots.height = dotsInX;

    cv::Size2i offset;
    offset.width = templateImage.cols / 10;
    offset.height = templateImage.rows / 10;

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
      std::cout << "Read:" << argv[i] << std::endl;

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

      unsigned long int maxArea = 10000;

      niftk::RingsPointDetector* detector1 = new niftk::RingsPointDetector(dots, offset);
      detector1->SetImage(&greyImage);
      detector1->SetImageScaleFactor(originalScaleFactors);
      detector1->SetTemplateImage(&templateImageGreyScale);
      detector1->SetReferenceImage(&referenceImageGreyScale);
      detector1->SetReferencePoints(referenceImageData.second);
      detector1->SetMaxAreaInPixels(maxArea);
      detector1->SetUseContours(true);
      detector1->SetUseInternalResampling(false);
      detector1->SetUseTemplateMatching(false);

      niftk::PointSet tmpPoints = detector1->GetPoints();
      if (tmpPoints.size() != dots.width * dots.height)
      {
        niftkNiftyCalThrow() << "Failed to extract points using contours from:" << argv[i];
      }

      maxArea = templateImageGreyScale.cols * templateImageGreyScale.rows;

      niftk::RingsPointDetector* detector2 = new niftk::RingsPointDetector(dots, offset);
      detector2->SetImageScaleFactor(warpedImageScaleFactors);
      detector2->SetTemplateImage(&templateImageGreyScale);
      detector2->SetReferenceImage(&referenceImageGreyScale);
      detector2->SetReferencePoints(referenceImageData.second);
      detector2->SetMaxAreaInPixels(maxArea);
      detector2->SetUseContours(false);
      detector2->SetUseInternalResampling(false);
      detector2->SetUseTemplateMatching(true);

      if (i-numberOfArgumentsBeforeImages < numberOfImagesPerSide)
      {
        std::shared_ptr<niftk::IPoint2DDetector> originalDetector(detector1);
        originalImagesLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
        dynamic_cast<niftk::RingsPointDetector*>(originalImagesLeft.back().first.get())->SetImage(&(originalImagesLeft.back().second));

        std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(detector2);
        imagesForWarpingLeft.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
        dynamic_cast<niftk::RingsPointDetector*>(imagesForWarpingLeft.back().first.get())->SetImage(&(imagesForWarpingLeft.back().second));
      }
      else
      {
        std::shared_ptr<niftk::IPoint2DDetector> originalDetector(detector1);
        originalImagesRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
        dynamic_cast<niftk::RingsPointDetector*>(originalImagesRight.back().first.get())->SetImage(&(originalImagesRight.back().second));

        std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(detector2);
        imagesForWarpingRight.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
        dynamic_cast<niftk::RingsPointDetector*>(imagesForWarpingRight.back().first.get())->SetImage(&(imagesForWarpingRight.back().second));
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
          optimise3D,
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
          flags
          );

    cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);

    std::cout << "niftkStereoIterativeRingsCalibration:(" << imageSize.width << "," << imageSize.height <<  ") "
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
              << leftToRightTranslationVector.at<double>(0,1) << " "
              << leftToRightTranslationVector.at<double>(0,2) << " "
              << rms(0, 0) << " "
              << rms(1, 0)
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
