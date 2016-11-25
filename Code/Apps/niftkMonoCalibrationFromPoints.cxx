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
#include <niftkTsaiCameraCalibration.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkPointUtilities.h>
#include <cv.h>
#include <list>
#include <cstdlib>

/**
* \file niftkMonoCalibrationFromPoints.cxx
* \brief Calibrate mono camera from pre-extracted points.
*/
int main(int argc, char ** argv)
{
  if (argc < 5)
  {
    std::cerr << "Usage: niftkMonoCalibrationFromPoints imageSizeX imageSizeY modelPoints.txt "
              << "imagePoints1.txt imagePoints2.txt ... imagePointsN.txt" << std::endl;
    return EXIT_FAILURE;
  }

  try
  {
    int sizeX = atoi(argv[1]);
    if (sizeX < 1)
    {
      niftkNiftyCalThrow() << "Invalid imageSizeX which should be >= 1";
    }
    int sizeY = atoi(argv[2]);
    if (sizeY < 1)
    {
      niftkNiftyCalThrow() << "Invalid sizeY which should be >= 1";
    }
    cv::Size2i imageSize(sizeX, sizeY);

    std::string modelFile = argv[3];
    niftk::Model3D model = niftk::LoadModel3D(modelFile);

    std::list<niftk::PointSet> points;

    for (int i = 4; i < argc; i++)
    {
      niftk::PointSet p = niftk::LoadPointSet(argv[i]);
      if (p.size() >= 4) // Deep within OpenCV lies a check for at least 4 points.
      {
        points.push_back(p);
      }
    }
    if (points.size() == 0)
    {
      niftkNiftyCalThrow() << "No valid points were read.";
    }

    cv::Mat intrinsic;
    cv::Mat distortion;

    double rms = 0;

    if (points.size() == 1)
    {
      // Can try Tsai 1987 calibration.

      cv::Point2d sensorDimensions(1,1);
      double sensorScaleInX = 1;
      cv::Mat rvec;
      cv::Mat tvec;

      if (niftk::ModelIsPlanar(model))
      {
        rms = niftk::TsaiMonoCoplanarCameraCalibration(model,
                                                       *(points.begin()),
                                                       imageSize,
                                                       sensorDimensions,
                                                       imageSize.width,
                                                       sensorScaleInX,
                                                       intrinsic,
                                                       distortion,
                                                       rvec,
                                                       tvec,
                                                       true // full optimisation
                                                       );
      }
      else
      {
        rms = niftk::TsaiMonoNonCoplanarCameraCalibration(model,
                                                          *(points.begin()),
                                                          imageSize,
                                                          sensorDimensions,
                                                          imageSize.width,
                                                          sensorScaleInX,
                                                          intrinsic,
                                                          distortion,
                                                          rvec,
                                                          tvec,
                                                          true // full optimisation
                                                         );
      }

      std::cout << "niftkMonoCalibrationFromPoints:(" << imageSize.width << "," << imageSize.height <<  ") "
                << points.size() << " "
                << sensorScaleInX << " "
                << intrinsic.at<double>(0,0) << " "
                << intrinsic.at<double>(1,1) << " "
                << intrinsic.at<double>(0,2) << " "
                << intrinsic.at<double>(1,2) << " "
                << distortion.at<double>(0,0) << " "
                << distortion.at<double>(0,1) << " "
                << distortion.at<double>(0,2) << " "
                << distortion.at<double>(0,3) << " "
                << distortion.at<double>(0,4) << " "
                << rvec.at<double>(0,0) << " "
                << rvec.at<double>(0,1) << " "
                << rvec.at<double>(0,2) << " "
                << tvec.at<double>(0,0) << " "
                << tvec.at<double>(0,1) << " "
                << tvec.at<double>(0,2) << " "
                << rms
                << std::endl;
    }
    else
    {
      // Can try Zhang 2000 calibration.

      std::vector<cv::Mat> rvecs;
      std::vector<cv::Mat> tvecs;

      rms = niftk::MonoCameraCalibration(model,
                                         points,
                                         imageSize,
                                         intrinsic,
                                         distortion,
                                         rvecs,
                                         tvecs
                                        );
      std::cout << "niftkMonoCalibrationFromPoints:(" << imageSize.width << "," << imageSize.height <<  ") "
                << points.size() << " "
                << intrinsic.at<double>(0,0) << " "
                << intrinsic.at<double>(1,1) << " "
                << intrinsic.at<double>(0,2) << " "
                << intrinsic.at<double>(1,2) << " "
                << distortion.at<double>(0,0) << " "
                << distortion.at<double>(0,1) << " "
                << distortion.at<double>(0,2) << " "
                << distortion.at<double>(0,3) << " "
                << distortion.at<double>(0,4) << " "
                << rms
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
