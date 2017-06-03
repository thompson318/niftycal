/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <niftkIOUtilities.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkCalibratedRenderingPipeline.h>
#include <cv.h>
#include <list>
#include <cstdlib>

/**
* \file niftkRenderCalibratedModel.cxx
* \brief Test harness to render a picture of a calibration pattern.
* \ingroup applications
*/
int main(int argc, char ** argv)
{
  if (argc != 10)
  {
    std::cerr << "Usage: niftkRenderCalibratedModel imageSizeX imageSizeY calibratedSizeX calibratedSizeY vtkPolyData.vtk texture.png "
              << "intrinsics3x3.txt extrinsics4x4.txt output.png" << std::endl;
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
    cv::Size2i windowSize(sizeX, sizeY);

    int calibratedSizeX = atoi(argv[3]);
    if (calibratedSizeX < 1)
    {
      niftkNiftyCalThrow() << "Invalid calibratedSizeX which should be >= 1";
    }
    int calibratedSizeY = atoi(argv[4]);
    if (calibratedSizeY < 1)
    {
      niftkNiftyCalThrow() << "Invalid calibratedSizeY which should be >= 1";
    }
    cv::Size2i calibratedSize(calibratedSizeX, calibratedSizeY);

    std::string modelFile = argv[5];
    std::string textureFile = argv[6];

    CalibratedRenderingPipeline p(windowSize,
                                  calibratedSize,
                                  modelFile,
                                  textureFile
                                  );

    cv::Mat intrinsics = niftk::LoadMatrix(argv[7]);
    cv::Mat extrinsics = niftk::LoadMatrix(argv[8]);

    cv::Matx44d cameraMatrix = extrinsics;

    p.SetIntrinsics(intrinsics);
    p.SetCameraToWorldMatrix(cameraMatrix);
    p.DumpScreen(argv[9]);

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
