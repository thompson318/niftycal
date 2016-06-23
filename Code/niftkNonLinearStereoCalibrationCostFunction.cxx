/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereoCalibrationCostFunction.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkMatrixUtilities.h"
#include "niftkPointUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereoCalibrationCostFunction::NonLinearStereoCalibrationCostFunction()
: m_RightHandPoints(nullptr)
, m_NumberOfRightHandValues(0)
{
}


//-----------------------------------------------------------------------------
NonLinearStereoCalibrationCostFunction::~NonLinearStereoCalibrationCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereoCalibrationCostFunction::SetRightHandPoints(std::list<PointSet>* const points,
                                                                const int& numDimensions)
{
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

  m_NumberOfRightHandValues = num * numDimensions;
  m_RightHandPoints = points;
  this->Modified();
}


//-----------------------------------------------------------------------------
unsigned int NonLinearStereoCalibrationCostFunction::GetNumberOfValues(void) const
{
  return m_NumberOfValues + m_NumberOfRightHandValues;
}


//-----------------------------------------------------------------------------
NonLinearStereoCalibrationCostFunction::MeasureType
NonLinearStereoCalibrationCostFunction::InternalGetValue(const ParametersType& parameters ) const
{
  if (m_Points->size() != m_RightHandPoints->size())
  {
    niftkNiftyCalThrow() << "Different number of left and right point sets.";
  }

  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  return result;
}

} // end namespace
