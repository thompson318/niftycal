/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearStereo3DCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearStereo3DCostFunction::NonLinearStereo3DCostFunction()
: m_RvecsLeft(nullptr)
, m_TvecsLeft(nullptr)
, m_LeftToRightRotationMatrix(nullptr)
, m_LeftToRightTranslationVector(nullptr)
{
}


//-----------------------------------------------------------------------------
NonLinearStereo3DCostFunction::~NonLinearStereo3DCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearStereo3DCostFunction::SetExtrinsics(std::vector<cv::Mat>* const rvecsLeft,
                                                  std::vector<cv::Mat>* const tvecsLeft,
                                                  cv::Mat* const leftToRightRotationMatrix,
                                                  cv::Mat* const leftToRightTranslationVector
                                                 )
{
  if (rvecsLeft == nullptr)
  {
    niftkNiftyCalThrow() << "Null left camera rotation vectors.";
  }

  if (tvecsLeft == nullptr)
  {
    niftkNiftyCalThrow() << "Null left camera translation vectors.";
  }

  if (leftToRightRotationMatrix == nullptr)
  {
    niftkNiftyCalThrow() << "Null leftToRightRotationMatrix.";
  }

  if (leftToRightTranslationVector == nullptr)
  {
    niftkNiftyCalThrow() << "Null leftToRightTranslationVector.";
  }

  if (leftToRightRotationMatrix->rows != 3 || leftToRightRotationMatrix->cols != 3)
  {
    niftkNiftyCalThrow() << "Left to Right rotation matrix should be 3x3, and its ("
                         << leftToRightRotationMatrix->cols << ", " << leftToRightRotationMatrix->rows << ")";
  }

  if (leftToRightTranslationVector->rows != 3 || leftToRightTranslationVector->cols != 1)
  {
    niftkNiftyCalThrow() << "Left to Right translation vector matrix should be 3x1, and its ("
                         << leftToRightTranslationVector->rows << ", " << leftToRightTranslationVector->cols << ")";
  }

  if (rvecsLeft->size() != tvecsLeft->size())
  {
    niftkNiftyCalThrow() << "Unequal extrinsic vectors: " << rvecsLeft->size()
                         << ", versus " << tvecsLeft->size();
  }

  m_RvecsLeft = rvecsLeft;
  m_TvecsLeft = tvecsLeft;
  m_LeftToRightRotationMatrix = leftToRightRotationMatrix;
  m_LeftToRightTranslationVector = leftToRightTranslationVector;
  this->Modified();
}


//-----------------------------------------------------------------------------
unsigned int NonLinearStereo3DCostFunction::GetNumberOfValues(void) const
{
  return this->GetNumberOfTriangulatablePoints(); // DONT DO THIS:  * 3; // for dx, dy, dz in 3D.
}

} // end namespace
