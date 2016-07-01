/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearCostFunction.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkMatrixUtilities.h"
#include "niftkPointUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearCostFunction::NonLinearCostFunction()
: m_Model(nullptr)
, m_Points(nullptr)
, m_NumberOfParameters(0)
, m_NumberOfValues(0)
{
}


//-----------------------------------------------------------------------------
NonLinearCostFunction::~NonLinearCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearCostFunction::SetModel(const Model3D* const model)
{
  if (model == nullptr)
  {
    niftkNiftyCalThrow() << "Model is NULL.";
  }

  m_Model = const_cast<Model3D*>(model);
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearCostFunction::SetPoints(const std::list<PointSet>* const points)
{
  if (points == nullptr)
  {
    niftkNiftyCalThrow() << "Points are NULL.";
  }


  unsigned int num = 0;
  std::list<PointSet>::const_iterator iter;
  for (iter = points->begin();
       iter != points->end();
       ++iter
       )
  {
    num += (*iter).size();
  }
  m_NumberOfValues = num;
  m_Points = const_cast<std::list<PointSet>*>(points);
  this->Modified();
}


//-----------------------------------------------------------------------------
unsigned int NonLinearCostFunction::GetNumberOfValues(void) const
{
  return m_NumberOfValues * 2; // multiply by 2 for dx, dy.
}


//-----------------------------------------------------------------------------
unsigned int NonLinearCostFunction::GetNumberOfParameters() const
{
  return m_NumberOfParameters;
}


//-----------------------------------------------------------------------------
void NonLinearCostFunction::GetDerivative(const ParametersType& parameters, DerivativeType& derivative ) const
{
  niftkNiftyCalThrow() << "Not implemented yet, use vnl derivative.";
}


//-----------------------------------------------------------------------------
double NonLinearCostFunction::GetRMS(const MeasureType& values) const
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
NonLinearCostFunction::MeasureType
NonLinearCostFunction::GetValue(const ParametersType& parameters ) const
{
  if (m_Model == nullptr)
  {
    niftkNiftyCalThrow() << "Model is null.";
  }
  if (m_Points == nullptr)
  {
    niftkNiftyCalThrow() << "Extracted points are null.";
  }
  if (m_Points->empty())
  {
    niftkNiftyCalThrow() << "No extracted points.";
  }
  return this->InternalGetValue(parameters);
}

} // end namespace
