/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkStereoCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-----------------------------------------------------------------------------
double StereoCameraCalibration(const Model3D& model,
                               const std::list<PointSet>& listOfLeftHandPointSets,
                               const std::list<PointSet>& listOfRightHandPointSets,
                               cv::Mat& intrinsicLeft,
                               cv::Mat& distortionLeft,
                               std::vector<cv::Mat>& rvecsLeft,
                               std::vector<cv::Mat>& tvecsLeft,
                               cv::Mat& intrinsicRight,
                               cv::Mat& distortionRight,
                               std::vector<cv::Mat>& rvecsRight,
                               std::vector<cv::Mat>& tvecsRight,
                               cv::Mat& left2RightRotation,
                               cv::Mat& left2RightTranslation,
                               cv::Mat& essentialMatrix,
                               cv::Mat& fundamentalMatrix
                              )
{
  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model should not be empty.";
  }

}

} // end namespace
