/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearHandEyeOptimiser.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearHandEyeOptimiser::NonLinearHandEyeOptimiser()
{
  m_CostFunction = niftk::NonLinearHandEyeCostFunction::New();
}


//-----------------------------------------------------------------------------
NonLinearHandEyeOptimiser::~NonLinearHandEyeOptimiser()
{

}


//-----------------------------------------------------------------------------
void NonLinearHandEyeOptimiser::SetModel(Model3D* const model)
{
  m_CostFunction->SetModel(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeOptimiser::SetPoints(std::list<PointSet>* const points)
{
  m_CostFunction->SetPoints(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeOptimiser::SetHandMatrices(std::list<cv::Matx44d>* const matrices)
{
  m_CostFunction->SetHandMatrices(matrices);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeOptimiser::SetEyeMatrices(std::list<cv::Matx44d>* const matrices)
{
  m_CostFunction->SetEyeMatrices(matrices);
  this->Modified();
}


//-----------------------------------------------------------------------------
double Optimise(cv::Matx44d& modelToWorld,
                cv::Matx44d& initialHandEye,
                cv::Mat& intrinsic,
                cv::Mat& distortion
               )
{

}

} // end namespace
