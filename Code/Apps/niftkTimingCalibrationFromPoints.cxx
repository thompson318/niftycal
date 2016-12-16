/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include <niftkIOUtilities.h>
#include <niftkTimingCalibration.h>
#include <niftkNiftyCalException.h>
#include <niftkNiftyCalExceptionMacro.h>

/**
* \file niftkTimingCalibrationFromPoints.cxx
* \brief Calibrate timing lag between a 2D signal and a 3D signal.
* \ingroup applications
*/
int main(int argc, char ** argv)
{
  if (argc != 3)
  {
    std::cerr << "Usage: niftkTimingCalibrationFromPoints imagePoints.txt trackerPoints.txt" << std::endl;
    return EXIT_FAILURE;
  }

  try
  {
    niftk::TimeSamples2D time2D = niftk::LoadTimeSamples2D(argv[1]);
    niftk::TimeSamples3D time3D = niftk::LoadTimeSamples3D(argv[2]);

    int lag = niftk::TimingCalibration(time3D, time2D);

    std::cout << "niftkTimingCalibrationFromPoints: " << lag << " (ms)" << std::endl;
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
