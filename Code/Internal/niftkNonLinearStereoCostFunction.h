/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoCostFunction_h
#define niftkNonLinearStereoCostFunction_h

#include "niftkNonLinearCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoCostFunction
* \brief Base class for stereo cost functions used to
* optimise either 2D reprojection error or 3D reconstruction error.
*/
class NonLinearStereoCostFunction : public NonLinearCostFunction
{

public:

  typedef NonLinearStereoCostFunction   Self;
  typedef NonLinearCostFunction         Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;

  typedef Superclass::ParametersType    ParametersType;
  typedef Superclass::DerivativeType    DerivativeType;
  typedef Superclass::MeasureType       MeasureType;

  void SetRightHandPoints(const std::list<PointSet>* const points);
  unsigned int GetNumberOfRightHandValues() const;
  unsigned int GetNumberOfTriangulatablePoints() const;

protected:

  NonLinearStereoCostFunction();
  virtual ~NonLinearStereoCostFunction();

  NonLinearStereoCostFunction(const NonLinearStereoCostFunction&); // Purposefully not implemented.
  NonLinearStereoCostFunction& operator=(const NonLinearStereoCostFunction&); // Purposefully not implemented.

  std::list<PointSet> *m_RightHandPoints;

private:

  unsigned int         m_NumberOfRightHandValues;
  unsigned int         m_NumberOfTriangulatablePoints;
};

} // end namespace

#endif
