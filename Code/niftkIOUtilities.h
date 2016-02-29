/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkIOUtilities_h
#define niftkIOUtilities_h

#include "niftkWin32ExportHeader.h"
#include "niftkTypes.h"

#include <iostream>

/**
* \file niftkIOUtilities.h
* \brief Defines various simple functions to read/write points/models/matrices as plain text files.
*/
namespace niftk
{

NIFTYCAL_WINEXPORT void SavePointSet(const PointSet& p, const std::string& fileName);
NIFTYCAL_WINEXPORT PointSet LoadPointSet(const std::string& fileName);

NIFTYCAL_WINEXPORT void SaveModel3D(const Model3D& m, const std::string& fileName);
NIFTYCAL_WINEXPORT Model3D LoadModel3D(const std::string& fileName);

NIFTYCAL_WINEXPORT void SaveMatrix(const cv::Mat& m, const std::string& fileName);
NIFTYCAL_WINEXPORT cv::Mat LoadMatrix(const std::string& fileName);

NIFTYCAL_WINEXPORT void SavePoints(const bool& all,
                                   const Model3D& m,
                                   const std::vector<PointSet>& p,
                                   const std::string& fileName
                                   );

NIFTYCAL_WINEXPORT void LoadPoints(const std::string& fileName,
                                   const Model3D& m,
                                   const std::vector<PointSet>& p
                                   );
} // end namespace

#endif
