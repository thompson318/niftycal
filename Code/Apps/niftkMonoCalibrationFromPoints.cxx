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
#include <cv.h>
#include <list>
#include <cstdlib>

/**
* \file niftkMonoCalibrationFromPoints.cxx
* \brief Calibrate mono camera from pre-extracted points.
*/
int main(int argc, char ** argv)
{
  if (argc < 6)
  {
    std::cerr << "Usage: niftkMonoCalibrationFromPoints imageSizeX imageSizeY modelPoints.txt "
              << "imagePoints1.txt imagePoints2.txt ... imagePointsN.txt" << std::endl;
    return EXIT_FAILURE;
  }

  int sizeX = atoi(argv[1]);
  int sizeY = atoi(argv[2]);
  std::string modelFile = argv[3];

  cv::Size2i imageSize(sizeX, sizeY);
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

  cv::Mat intrinsic;
  cv::Mat distortion;
  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;

  double rms = niftk::MonoCameraCalibration(model,
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

  return EXIT_SUCCESS;
}
