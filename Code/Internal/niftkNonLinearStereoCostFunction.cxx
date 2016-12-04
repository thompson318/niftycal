/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkPointUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoCostFunction::NonLinearStereoCostFunction()
: m_NumberOfRightHandValues(0)
, m_NumberOfTriangulatablePoints(0)
{
}


//-----------------------------------------------------------------------------
NonLinearStereoCostFunction::~NonLinearStereoCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoCostFunction::SetRightHandPoints(const std::list<PointSet>* const points)
{
  if (m_Points->size() == 0)
  {
    niftkNiftyCalThrow() << "No left hand points present. Please set them first.";
  }
  if (points == nullptr)
  {
    niftkNiftyCalThrow() << "Null right hand points.";
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
  m_NumberOfRightHandValues = num;
  m_RightHandPoints = const_cast<std::list<PointSet>*>(points);
  m_NumberOfTriangulatablePoints = niftk::GetNumberOfTriangulatablePoints(*m_Model, *m_Points, *m_RightHandPoints);

  this->Modified();
}


//-----------------------------------------------------------------------------
unsigned int NonLinearStereoCostFunction::GetNumberOfRightHandValues(void) const
{
  return m_NumberOfRightHandValues;
}


//-----------------------------------------------------------------------------
unsigned int NonLinearStereoCostFunction::GetNumberOfTriangulatablePoints() const
{
  return m_NumberOfTriangulatablePoints;
}

} // end namespace
