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

#include <vtkRenderWindow.h>
#include <cv.h>
#include <highgui.h>
#include <list>
#include <cstdlib>

#include <QVTKWidget.h>
#include <QApplication>

/**
* \file niftkRenderCalibratedModel.cxx
* \brief Test harness to render a picture of a calibration pattern.
* \ingroup applications
*/
int main(int argc, char ** argv)
{
  if (argc != 10 && argc != 11)
  {
    std::cerr << "Usage: niftkRenderCalibratedModel imageSizeX imageSizeY calibratedSizeX calibratedSizeY vtkPolyData.vtk texture.png "
              << "intrinsics3x3.txt extrinsics4x4.txt [leftToRightExtrinsics4x4.txt] output.png" << std::endl;
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

    QApplication app(argc,argv);

    QVTKWidget *widget = new QVTKWidget();
    widget->show();
    widget->resize(windowSize.width, windowSize.height);

    vtkRenderWindow *window = widget->GetRenderWindow();
    window->DoubleBufferOff();
    window->GetInteractor()->Disable();

    p.ConnectToRenderWindow(window);

    cv::Mat intrinsics = niftk::LoadMatrix(argv[7]);
    p.SetIntrinsics(intrinsics);

    cv::Mat extrinsics = niftk::LoadMatrix(argv[8]);    
    p.SetWorldToCameraMatrix(extrinsics);

    cv::Matx44d leftToRight = cv::Matx44d::eye();
    if (argc == 11)
    {
      leftToRight = niftk::LoadMatrix(argv[9]);
      p.SetLeftToRightMatrix(leftToRight);
    }

    if (argc == 10)
    {
      cv::Mat tmp;
      p.CopyScreen(tmp);
      cv::imwrite(argv[9], tmp);
    }
    else if (argc == 11)
    {
      cv::Mat tmp;
      p.CopyScreen(tmp);
      cv::imwrite(argv[10], tmp);
    }
    else
    {
      niftkNiftyCalThrow() << "Incorrect number of parameters, "
                           << "which at this point, must be a programming bug.";
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
