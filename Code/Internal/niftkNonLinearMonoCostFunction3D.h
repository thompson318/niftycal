/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMonoCostFunction3D_h
#define niftkNonLinearMonoCostFunction3D_h

#include "niftkNonLinearCostFunction.h"

namespace niftk
{

/**
* \class NonLinearMonoCostFunction3D
* \brief Base class for non-linear mono cost functions used to optimise 3D reconstruction error.
*/
class NonLinearMonoCostFunction3D : public NonLinearCostFunction
{

public:

  typedef NonLinearMonoCostFunction3D   Self;
  typedef NonLinearCostFunction         Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;

  typedef Superclass::ParametersType    ParametersType;
  typedef Superclass::DerivativeType    DerivativeType;
  typedef Superclass::MeasureType       MeasureType;

  virtual void SetPoints(const std::list<PointSet>* const points);
  virtual unsigned int GetNumberOfValues(void) const ITK_OVERRIDE;

protected:

  NonLinearMonoCostFunction3D();
  virtual ~NonLinearMonoCostFunction3D();

  NonLinearMonoCostFunction3D(const NonLinearMonoCostFunction3D&); // Purposefully not implemented.
  NonLinearMonoCostFunction3D& operator=(const NonLinearMonoCostFunction3D&); // Purposefully not implemented.

  std::list<PointSet> m_FakeRightHandPoints;

private:

  unsigned int        m_NumberOfTriangulatablePoints;

};

} // end namespace

#endif
