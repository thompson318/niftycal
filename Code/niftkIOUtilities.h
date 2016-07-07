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
#include "niftkNiftyCalTypes.h"
#include <list>

/**
* \file niftkIOUtilities.h
* \brief Utilities to read/write points/models/matrices as plain text files.
*
* If PointSet/Model3D has zero points, we read/write an empty file. Its not an error.
*/
namespace niftk
{

NIFTYCAL_WINEXPORT void SavePointSet(const PointSet& p, const std::string& fileName);
NIFTYCAL_WINEXPORT PointSet LoadPointSet(const std::string& fileName);

NIFTYCAL_WINEXPORT void SaveModel3D(const Model3D& m, const std::string& fileName);
NIFTYCAL_WINEXPORT Model3D LoadModel3D(const std::string& fileName);

NIFTYCAL_WINEXPORT void SaveMatrix(const cv::Mat& m, const std::string& fileName);
NIFTYCAL_WINEXPORT cv::Mat LoadMatrix(const std::string& fileName);

NIFTYCAL_WINEXPORT void SavePoints(const Model3D& m,
                                   const std::list<PointSet>& p,
                                   const std::string& fileName
                                   );

NIFTYCAL_WINEXPORT void LoadPoints(const std::string& fileName,
                                   const Model3D& m,
                                   const std::list<PointSet>& p
                                   );

/**
* \brief Saves NifTK format intrinsics file.
*
* This output format is a 3x3 matrix, followed by 1x[4,5] distortion coefficients as a row vector.
*/
NIFTYCAL_WINEXPORT void SaveNifTKIntrinsics(const cv::Mat& intrinsics,
                                            const cv::Mat& distortion,
                                            const std::string& fileName
                                           );

/**
* \brief Saves NifTK format right-to-left stereo extrinsics transform.
*
* Note: OpenCV and NiftyCal naturally uses left-to-right. NifTK uses right-to-left.
* This output form is a 3x3 rotation matrix, followed by a translation as a 1x3 row vector.
*/
NIFTYCAL_WINEXPORT void SaveNifTKStereoExtrinsics(const cv::Mat& leftToRightRotationMatrix,
                                                  const cv::Mat& leftToRightTranslationVector,
                                                  const std::string& fileName
                                                 );


NIFTYCAL_WINEXPORT void LoadNifTKStereoExtrinsics(const std::string& fileName,
                                                  cv::Mat& leftToRightRotationMatrix,
                                                  cv::Mat& leftToRightTranslationVector
                                                  );

/**
* \brief Does Rodrigues decomposition, and saves 6 parameters to file.
*/
NIFTYCAL_WINEXPORT void SaveRigidParams(const cv::Matx44d& matrix,
                                        const std::string& fileName
                                        );


NIFTYCAL_WINEXPORT void SaveRigidParams(const cv::Mat& rotationVector,
                                        const cv::Mat& translationVector,
                                        const std::string& fileName
                                        );

/**
* \brief Writes a 4x4 matrix to file as 4 numbers per line.
*/
NIFTYCAL_WINEXPORT void Save4x4Matrix(const cv::Mat& leftToRightRotationMatrix,
                                      const cv::Mat& leftToRightTranslationVector,
                                      const std::string& fileName
                                     );

/**
* \brief Writes a 4x4 matrix to file as 4 numbers per line.
*/
NIFTYCAL_WINEXPORT void Save4x4Matrix(const cv::Matx44d& matrix,
                                      const std::string& fileName
                                     );

} // end namespace

#endif
