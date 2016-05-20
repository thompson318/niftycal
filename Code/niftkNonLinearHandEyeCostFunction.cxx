/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearHandEyeCostFunction.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::NonLinearHandEyeCostFunction()
: m_Model(nullptr)
, m_Points(nullptr)
, m_HandMatrices(nullptr)
, m_EyeMatrices(nullptr)
, m_NumberOfParameters(0)
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
  unsigned int numberOfValues = 0;

  std::list<PointSet>::const_iterator iter;
  for (iter = m_Points->begin();
       iter != m_Points->end();
       ++iter
       )
  {
    numberOfValues += (*iter).size();
  }
  return numberOfValues * 2; // For each point, we have deltaX and deltaY.
}


//-----------------------------------------------------------------------------
unsigned int NonLinearHandEyeCostFunction::GetNumberOfParameters() const
{
  return m_NumberOfParameters;
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::GetDerivative(const ParametersType& parameters, DerivativeType& derivative ) const
{
  niftkNiftyCalThrow() << "Not implemented yet, use vnl derivative.";
}


//-----------------------------------------------------------------------------
double NonLinearHandEyeCostFunction::GetRMS(const MeasureType& values) const
{
  double rms = 0;
  if (values.GetSize() == 0)
  {
    return rms;
  }

  for (unsigned int i = 0; i < values.GetSize(); i++)
  {
    rms += (values[i] * values[i]);
  }
  rms /= static_cast<double>(values.GetSize());
  return sqrt(rms);
}


//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::MeasureType
NonLinearHandEyeCostFunction::GetValue(const ParametersType& parameters ) const
{
  MeasureType result;
  result.SetSize(this->GetNumberOfValues());
  result.Fill(0);
  return result;
}


} // end namespace
