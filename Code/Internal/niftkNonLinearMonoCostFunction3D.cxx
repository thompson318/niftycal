/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearMonoCostFunction3D.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearMonoCostFunction3D::NonLinearMonoCostFunction3D()
: m_NumberOfTriangulatablePoints(0)
{
}


//-----------------------------------------------------------------------------
NonLinearMonoCostFunction3D::~NonLinearMonoCostFunction3D()
{
}


//-----------------------------------------------------------------------------
unsigned int NonLinearMonoCostFunction3D::GetNumberOfValues(void) const
{
  return m_NumberOfTriangulatablePoints * 3; // multiply by 3 for dx, dy, dz.
}


//-----------------------------------------------------------------------------
void NonLinearMonoCostFunction3D::SetPoints(const std::list<PointSet>* const points)
{
  NonLinearCostFunction::SetPoints(points);

  if(m_Model == nullptr || m_Model->size() == 0)
  {
    niftkNiftyCalThrow() << "Model not set yet. Please set it first.";
  }

  std::list<PointSet>::const_iterator iter;
  iter = points->begin();
  iter++; // start on 2nd item.

  while(iter != points->end())
  {
    m_FakeRightHandPoints.push_back(*iter);
    iter++;
  }
  m_FakeRightHandPoints.push_back(*(points->begin()));
  m_NumberOfTriangulatablePoints = niftk::GetNumberOfTriangulatablePoints(*m_Model, *m_Points, m_FakeRightHandPoints);
}

} // end namespace
