/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkMonoCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-----------------------------------------------------------------------------
double MonoCameraCalibration(const Model3D& model,
                             const std::list<PointSet>& listOfPointSets,
                             cv::Matx33d& intrinsic,
                             cv::Matx14d& distortion,
                             std::list<cv::Matx44d>& extrinsics
                             )
{
  if (model.size() == 0)
  {
    niftkNiftyCalThrow() << "Model should not be empty.";
  }
  if (listOfPointSets.size() < 2)
  {
    niftkNiftyCalThrow() << "Should have at least two views of calibration points.";
  }

  double rms = 0;

  // Initialise matrices.
  intrinsic = cv::Matx33d::zeros();
  distortion = cv::Matx14d::zeros();
  cv::Matx44d id = cv::Matx44d::eye();

  // Remember, each PointSet in listOfPointSets can have
  // a different number of points, and also each PointSet
  // can have different point IDs. Furthermore, assume
  // that point IDs in a given PointSet may not be valid
  // i.e. they are not contained in Model3D. So, we need
  // to extract only the ones that match in 2D and 3D.



  return rms;
}

} // end namespace
