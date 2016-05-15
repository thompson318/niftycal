/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkMatrixtilities_h
#define niftkMatrixtilities_h

#include "niftkWin32ExportHeader.h"

/**
* \file niftkMatrixUtilities.h
* \brief Various matrix related functions.
*/
namespace niftk
{

NIFTYCAL_WINEXPORT cv::Matx44d RotationAndTranslationToMatrix(const cv::Mat& rotationMatrix3x3,
                                                              const cv::Mat& translationVector1x3);

NIFTYCAL_WINEXPORT cv::Matx44d RodriguesToMatrix(const cv::Mat& rotationVector1x3,
                                                 const cv::Mat& translationVector1x3);


/**
* \brief Averages a list of rigid body matrices.
*
* Originally implemented by Steve Thompson in NifTK, but converted
* to use lists and cv::Matx44d for NiftyCal.
*/
NIFTYCAL_WINEXPORT cv::Matx44d AverageMatrices(const std::list<cv::Matx44d >&);

} // end namespace

#endif
