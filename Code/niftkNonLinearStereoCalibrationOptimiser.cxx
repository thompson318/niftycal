/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoCalibrationOptimiser.h"
#include "niftkMatrixUtilities.h"
#include "niftkNiftyCalExceptionMacro.h"
#include <itkLevenbergMarquardtOptimizer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoCalibrationOptimiser::NonLinearStereoCalibrationOptimiser()
{
  m_CostFunction = niftk::NonLinearStereoCalibrationCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearStereoCalibrationOptimiser::~NonLinearStereoCalibrationOptimiser()
{

}


//-----------------------------------------------------------------------------
void NonLinearStereoCalibrationOptimiser::SetModel(Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoCalibrationOptimiser::SetPoints(std::list<PointSet>* const points)
{
  m_CostFunction->SetPoints(points, 3);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearStereoCalibrationOptimiser::SetRightHandPoints(std::list<PointSet>* const points)
{
  m_CostFunction->SetRightHandPoints(points, 3);
  this->Modified();
}


//-----------------------------------------------------------------------------
double NonLinearStereoCalibrationOptimiser::Optimise(cv::Mat& leftIntrinsic,
                                                     cv::Mat& leftDistortion,
                                                     cv::Mat& rightIntrinsic,
                                                     cv::Mat& rightDistortion,
                                                     cv::Mat& leftToRightRotationMatrix,
                                                     cv::Mat& leftToRightTranslationVector
                                                    )
{
  double finalRMS = 0;
  return finalRMS;
}

} // end namespace
