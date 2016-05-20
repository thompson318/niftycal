/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearHandEyeCostFunction.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::NonLinearHandEyeCostFunction()
: m_Model(nullptr)
, m_Points(nullptr)
, m_HandMatrices(nullptr)
, m_EyeMatrices(nullptr)
{

}


//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::~NonLinearHandEyeCostFunction()
{

}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::SetModel(Model3D* const model)
{
  m_Model = model;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::SetPoints(std::list<PointSet>* const points)
{
  m_Points = points;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::SetHandMatrices(std::list<cv::Matx44d>* const matrices)
{
  m_HandMatrices = matrices;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::SetEyeMatrices(std::list<cv::Matx44d>* const matrices)
{
  m_EyeMatrices = matrices;
  this->Modified();
}


//-----------------------------------------------------------------------------
unsigned int NonLinearHandEyeCostFunction::GetNumberOfValues(void) const
{
  return 0;
}


//-----------------------------------------------------------------------------
unsigned int NonLinearHandEyeCostFunction::GetNumberOfParameters() const
{
  return 0;
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::GetDerivative(const ParametersType& parameters, DerivativeType& derivative ) const
{

}


//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::MeasureType
NonLinearHandEyeCostFunction::GetValue(const ParametersType& parameters ) const
{
  MeasureType result;
  return result;
}


} // end namespace
