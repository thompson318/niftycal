/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <niftkIOUtilities.h>
#include <niftkPoseFromPoints.h>
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <cv.h>
#include <list>
#include <cstdlib>

/**
* \file niftkPoseFromPoints.cxx
* \brief Calculates Camera Pose from pre-extracted points.
*/
int main(int argc, char ** argv)
{
  if (argc < 5)
  {
    std::cerr << "Usage: niftkPoseFromPoints modelPoints.txt intrinsic.txt distortion"
              << "imagePoints1.txt imagePoints2.txt ... imagePointsN.txt" << std::endl;
    return EXIT_FAILURE;
  }

  try
  {
    std::string modelFile = argv[1];
    niftk::Model3D model = niftk::LoadModel3D(modelFile);

    std::string intrinsicFileName = argv[2];
    std::string distortionFileName = argv[3];

    cv::Mat intrinsic = niftk::LoadMatrix(intrinsicFileName);
    cv::Mat distortion = niftk::LoadMatrix(distortionFileName);
    std::list<niftk::PointSet> points;

    for (int i = 4; i < argc; i++)
    {
      niftk::PointSet p = niftk::LoadPointSet(argv[i]);
      if (p.size() >= 4) // Deep within OpenCV lies a check for at least 4 points.
      {
        points.push_back(p);
      }
    }

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    niftk::PoseFromPoints ( model,
                            points,
                            intrinsic,
                            distortion,
                            rvecs,
                            tvecs
                          );

    for ( unsigned int i = 0 ; i < rvecs.size() ; ++ i )
    {
      // Convert rvec in Rodrigues notation to pitch, yaw, roll
      cv::Mat eulerAngles = niftk::RodriguesToEulerAngles ( rvecs[i] );

      std::cout << eulerAngles.at<double>(0,0) * 180.0 / CV_PI << " , "
                << eulerAngles.at<double>(1,0) * 180.0 / CV_PI << " , "
                << eulerAngles.at<double>(2,0) * 180.0 / CV_PI << " , "
                << tvecs[i].at<double>(0,0) << " , "
                << tvecs[i].at<double>(0,1) << " , "
                << tvecs[i].at<double>(0,2)
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
