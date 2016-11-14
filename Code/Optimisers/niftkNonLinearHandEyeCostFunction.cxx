/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearHandEyeCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::NonLinearHandEyeCostFunction()
: m_HandMatrices(nullptr)
{
}


//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::~NonLinearHandEyeCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::SetHandMatrices(std::list<cv::Matx44d>* const matrices)
{
  if (matrices == nullptr)
  {
    niftkNiftyCalThrow() << "Hand matrices are NULL.";
  }

  m_HandMatrices = matrices;
  this->Modified();
}


//-----------------------------------------------------------------------------
std::list<cv::Matx44d>* NonLinearHandEyeCostFunction::GetHandMatrices() const
{
  return m_HandMatrices;
}


//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::MeasureType
NonLinearHandEyeCostFunction::GetValue(const ParametersType& parameters ) const
{
  if (m_HandMatrices == nullptr)
  {
    niftkNiftyCalThrow() << "Hand matrices are null.";
  }
  if (m_HandMatrices->empty())
  {
    niftkNiftyCalThrow() << "No tracking matrices.";
  }
  if (m_Points->size() != m_HandMatrices->size())
  {
    niftkNiftyCalThrow() << "Different number of point sets and hand matrices.";
  }
  return NonLinearCostFunction::GetValue(parameters);
}

} // end namespace
