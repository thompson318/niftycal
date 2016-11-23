/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearTsaiCostFunction.h"
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearTsaiCostFunction::NonLinearTsaiCostFunction()
{
}


//-----------------------------------------------------------------------------
NonLinearTsaiCostFunction::~NonLinearTsaiCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearTsaiCostFunction::ComputeErrorValues(const niftk::Model3D& model,
                                                   const niftk::PointSet& points,
                                                   const cv::Matx44d& extrinsic,
                                                   const cv::Mat& intrinsic,
                                                   const cv::Mat& distortion,
                                                   NonLinearTsaiCostFunction::MeasureType& errorValues
                                                  ) const
{
  unsigned int totalPointCounter = 0;
  std::vector<cv::Point2f> observed(points.size());
  std::vector<cv::Point2f> projected(points.size());
  std::vector<niftk::NiftyCalIdType> ids(points.size());

  niftk::ProjectMatchingPoints(model,
                               points,
                               extrinsic,
                               intrinsic,
                               distortion,
                               observed,
                               projected,
                               ids
                              );

  for (unsigned int i = 0; i < observed.size(); i++)
  {
    errorValues[totalPointCounter++] = (observed[i].x - projected[i].x);
    errorValues[totalPointCounter++] = (observed[i].y - projected[i].y);
  }
}

} // end namespace
