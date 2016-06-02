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
#include <niftkAprilTagsPointDetector.h>
#include <cv.h>
#include <highgui.h>
#include <cstdlib>

/**
* \file niftkMonoIterativeAprilTagsCalibration.cxx
* \brief Calibrate mono camera, using AprilTag features, and
* the Dutta-2009 iterative optimisation algorithm.
*/
int main(int argc, char ** argv)
{
  if (argc < 11)
  {
    std::cerr << "Usage: niftkMonoIterativeAprilTagsCalibration modelPoints.txt "
              << " referenceImage.png referencePoints tagFamily rescaleX rescaleY zeroDistortion"
              << "image1.png image2.png ... imageN.txt" << std::endl;
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

  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > originalImages;
  std::list< std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat> > imagesForWarping;

  for (int i = 8; i < argc; i++)
  {
    cv::Mat image = cv::imread(argv[i]);
    if (i == 8)
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

    niftk::AprilTagsPointDetector* detector1 = new niftk::AprilTagsPointDetector(true, tagFamily, 0, 0.8);
    detector1->SetImageScaleFactor(scaleFactors);

    std::shared_ptr<niftk::IPoint2DDetector> originalDetector(detector1);
    originalImages.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(originalDetector, greyImage));
    dynamic_cast<niftk::AprilTagsPointDetector*>(originalImages.back().first.get())->SetImage(&(originalImages.back().second));

    cv::Mat greyImageClone = greyImage.clone();

    niftk::AprilTagsPointDetector* detector2 = new niftk::AprilTagsPointDetector(true, tagFamily, 0, 0.8);
    detector2->SetImageScaleFactor(scaleFactors);

    std::shared_ptr<niftk::IPoint2DDetector> warpedDetector(detector2);
    imagesForWarping.push_back(std::pair<std::shared_ptr<niftk::IPoint2DDetector>, cv::Mat>(warpedDetector, greyImageClone));
    dynamic_cast<niftk::AprilTagsPointDetector*>(imagesForWarping.back().first.get())->SetImage(&(imagesForWarping.back().second));
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

  std::cout << "niftkMonoIterativeAprilTagsCalibration:(" << imageSize.width << "," << imageSize.height <<  ") "
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
