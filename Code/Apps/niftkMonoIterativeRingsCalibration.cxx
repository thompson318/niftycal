/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <niftkIOUtilities.h>
#include <niftkIterativeMonoCameraCalibration.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkRingsPointDetector.h>
#include <cv.h>
#include <highgui.h>
#include <cstdlib>

/**
* \file niftkMonoIterativeRingsCalibration.cxx
* \brief Calibrate mono camera, using Ring features, and
* the Dutta-2009 iterative optimisation algorithm.
*/
int main(int argc, char ** argv)
{
  if (argc < 13)
  {
    std::cerr << "Usage: niftkMonoIterativeRingsCalibration modelPoints.txt "
              << " referenceImage.png referencePoints templateImage dotsInX dotsInY rescaleX rescaleY zeroDistortion"
              << "image1.png image2.png ... imageN.txt" << std::endl;
    return EXIT_FAILURE;
  }

  for (int i = 0; i < argc; i++)
  {
    std::cout << "Arg[" << i << "]=" << argv[i] << std::endl;
  }
  std::string modelFile = argv[1];
  std::string referenceImageFile = argv[2];
  std::string referencePointsFile = argv[3];
  std::string templateImageFile = argv[4];
  int dotsInX = atoi(argv[5]);
  int dotsInY = atoi(argv[6]);
  float rescaleX = atof(argv[7]);
  float rescaleY = atof(argv[8]);
  int   zeroDistortion = atoi(argv[9]);

  // This is passed through to cv::findCirclesGrid, for which the documentation
  // says "patternSize = Size(points_per_row, points_per_colum)"
  // which is the opposite way round to the normal cv::Size2i constructor.
  cv::Size2i dots;
  dots.width = dotsInY;
  dots.height = dotsInX;

  cv::Size2i offset;
  offset.width = 5;
  offset.height = 5;

  cv::Point2d scaleFactors;
  scaleFactors.x = rescaleX;
  scaleFactors.y = rescaleY;

  std::cout << "Info: dots=" << dots << ", offset=" << offset << ", scaleFactors=" << scaleFactors << std::endl;

  cv::Size2i imageSize;
  niftk::Model3D model = niftk::LoadModel3D(modelFile);

  cv::Mat referenceImage = cv::imread(referenceImageFile);
  cv::Mat referenceImageGreyScale;
  cv::cvtColor(referenceImage, referenceImageGreyScale, CV_BGR2GRAY);

  cv::Mat templateImage = cv::imread(templateImageFile);
  cv::Mat templateImageGreyScale;
  cv::cvtColor(templateImage, templateImageGreyScale, CV_BGR2GRAY);

  std::pair< cv::Mat, niftk::PointSet> referenceImageData;
  referenceImageData.first = referenceImageGreyScale;
  referenceImageData.second = niftk::LoadPointSet(referencePointsFile);

  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImages;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarping;

  for (int i = 10; i < argc; i++)
  {
    cv::Mat image = cv::imread(argv[i]);
    std::cout << "Read:" << argv[i] << std::endl;

    if (i == 10)
    {
      imageSize.width = image.cols;
      imageSize.height = image.rows;
    }
    else
    {
      if (image.cols != imageSize.width
          || image.rows != imageSize.height
          )
      {
        niftkNiftyCalThrow() << "Invalid image size:" << image.cols << "x" << image.rows << std::endl;
      }
    }
    cv::Mat greyImage;
    cv::cvtColor(image, greyImage, CV_BGR2GRAY);

    unsigned long int maxArea = 10000;

    niftk::RingsPointDetector* detector1 = new niftk::RingsPointDetector(dots, offset);
    detector1->SetImageScaleFactor(scaleFactors);
    detector1->SetTemplateImage(&templateImageGreyScale);
    detector1->SetReferenceImage(&referenceImageGreyScale);
    detector1->SetReferencePoints(referenceImageData.second);
    detector1->SetMaxAreaInPixels(maxArea);
    detector1->SetUseContours(true);
    detector1->SetUseInternalResampling(false);
    detector1->SetUseTemplateMatching(false);

    std::shared_ptr<niftk::IPoint2DDetector> originalDetector(detector1);
    originalImages.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
    dynamic_cast<niftk::RingsPointDetector*>(originalImages.back().first.get())->SetImage(&(originalImages.back().second));

    niftk::PointSet tmpPoints = detector1->GetPoints();
    if (tmpPoints.size() != dots.width * dots.height)
    {
      niftkNiftyCalThrow() << "Failed to extract points using contours from:" << argv[i];
    }

    cv::Mat greyImageClone = greyImage.clone();

    maxArea = templateImageGreyScale.cols * templateImageGreyScale.rows;

    niftk::RingsPointDetector* detector2 = new niftk::RingsPointDetector(dots, offset);
    detector2->SetImageScaleFactor(scaleFactors);
    detector2->SetTemplateImage(&templateImageGreyScale);
    detector2->SetReferenceImage(&referenceImageGreyScale);
    detector2->SetReferencePoints(referenceImageData.second);
    detector2->SetMaxAreaInPixels(maxArea);
    detector2->SetUseContours(true);
    detector2->SetUseInternalResampling(true);
    detector2->SetUseTemplateMatching(true);

    tmpPoints = detector1->GetPoints();
    if (tmpPoints.size() != dots.width * dots.height)
    {
      niftkNiftyCalThrow() << "Failed to extract points using template matching from:" << argv[i];
    }

    detector2->SetUseContours(false);
    detector2->SetUseInternalResampling(false);
    detector2->SetUseTemplateMatching(true);

    std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(detector2);
    imagesForWarping.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
    dynamic_cast<niftk::RingsPointDetector*>(imagesForWarping.back().first.get())->SetImage(&(imagesForWarping.back().second));
  }

  cv::Mat intrinsic;
  cv::Mat distortion;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  int flags = 0;
  if (zeroDistortion != 0)
  {
    flags = cv::CALIB_ZERO_TANGENT_DIST
        | cv::CALIB_FIX_K1 | cv::CALIB_FIX_K2
        | cv::CALIB_FIX_K3 | cv::CALIB_FIX_K4
        | cv::CALIB_FIX_K5 | cv::CALIB_FIX_K6;
  }

  double rms = niftk::IterativeMonoCameraCalibration(
    model,
    referenceImageData,
    originalImages,
    imagesForWarping,
    imageSize,
    intrinsic,
    distortion,
    rvecs,
    tvecs,
    flags
    );

  std::cout << "niftkMonoIterativeRingsCalibration:(" << imageSize.width << "," << imageSize.height <<  ") "
            << originalImages.size() << " "
            << intrinsic.at<double>(0,0) << " "
            << intrinsic.at<double>(1,1) << " "
            << intrinsic.at<double>(0,2) << " "
            << intrinsic.at<double>(1,2) << " "
            << distortion.at<double>(0,0) << " "
            << distortion.at<double>(0,1) << " "
            << distortion.at<double>(0,2) << " "
            << distortion.at<double>(0,3) << " "
            << rms
            << std::endl;

  return EXIT_SUCCESS;
}
