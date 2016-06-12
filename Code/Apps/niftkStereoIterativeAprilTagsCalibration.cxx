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
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkAprilTagsPointDetector.h>
#include <cv.h>
#include <highgui.h>
#include <cstdlib>

/**
* \file niftkStereoIterativeAprilTagsCalibration.cxx
* \brief Calibrate mono camera, using AprilTag features, and
* the Dutta-2009 iterative optimisation algorithm.
*/
int main(int argc, char ** argv)
{
  if (argc < 14)
  {
    std::cerr << "Usage: niftkStereoIterativeAprilTagsCalibration modelPoints.txt "
              << " referenceImage.png referencePoints tagFamily rescaleX rescaleY zeroDistortion"
              << "leftImage1.png leftImage2.png ... leftImageN.txt"
              << "rightImage1.png rightImage2.png ... rightImageN.txt"
              << std::endl;
    return EXIT_FAILURE;
  }

  int numberOfArgumentsBeforeImages = 8;
  int numberOfImagesPerSide = (argc-numberOfArgumentsBeforeImages)/2;

  if ((argc - numberOfArgumentsBeforeImages)%2 != 0)
  {
    std::cerr << "Expected an even number of image files" << std::endl;
    return EXIT_FAILURE;
  }

  std::string modelFile = argv[1];
  std::string referenceImageFile = argv[2];
  std::string referencePointsFile = argv[3];
  std::string tagFamily = argv[4];
  float rescaleX = atof(argv[5]);
  float rescaleY = atof(argv[6]);
  int   zeroDistortion = atoi(argv[7]);

  cv::Point2d scaleFactors;
  scaleFactors.x = rescaleX;
  scaleFactors.y = rescaleY;

  cv::Size2i imageSize;
  niftk::Model3D model = niftk::LoadModel3D(modelFile);

  cv::Mat referenceImage = cv::imread(referenceImageFile);
  cv::Mat referenceImageGreyScale;
  cv::cvtColor(referenceImage, referenceImageGreyScale, CV_BGR2GRAY);

  std::pair< cv::Mat, niftk::PointSet> referenceImageData;
  referenceImageData.first = referenceImageGreyScale;
  referenceImageData.second = niftk::LoadPointSet(referencePointsFile);

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

    niftk::AprilTagsPointDetector* detector1 = new niftk::AprilTagsPointDetector(true, tagFamily, 0, 0.8);
    detector1->SetImageScaleFactor(scaleFactors);

    niftk::AprilTagsPointDetector* detector2 = new niftk::AprilTagsPointDetector(true, tagFamily, 0, 0.8);
    detector2->SetImageScaleFactor(scaleFactors);

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

  int flags = 0;
  if (zeroDistortion != 0)
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
  cv::Mat leftToRightTranslationVector;

  double rms = niftk::IterativeStereoCameraCalibration(
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

  cv::Mat rvec;
  cv::Rodrigues(leftToRightRotationMatrix, rvec);

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
            << rvec.at<double>(0,0) << " "
            << rvec.at<double>(0,1) << " "
            << rvec.at<double>(0,2) << " "
            << leftToRightTranslationVector.at<double>(0,0) << " "
            << leftToRightTranslationVector.at<double>(0,1) << " "
            << leftToRightTranslationVector.at<double>(0,2) << " "
            << rms
            << std::endl;

  return EXIT_SUCCESS;
}
