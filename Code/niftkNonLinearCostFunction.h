/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearCostFunction_h
#define niftkNonLinearCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"
#include <cv.h>
#include <itkMultipleValuedCostFunction.h>

namespace niftk
{

/**
* \class NonLinearCostFunction
* \brief Base class for non-linear cost functions used to
* optimise 2D reprojection or 3D reconstruction error.
*/
class NIFTYCAL_WINEXPORT NonLinearCostFunction :
    public itk::MultipleValuedCostFunction
{

public:

  typedef NonLinearCostFunction           Self;
  typedef itk::MultipleValuedCostFunction Superclass;
  typedef itk::SmartPointer<Self>         Pointer;
  typedef itk::SmartPointer<const Self>   ConstPointer;

  typedef Superclass::ParametersType      ParametersType;
  typedef Superclass::DerivativeType      DerivativeType;
  typedef Superclass::MeasureType         MeasureType;

  void SetModel(Model3D* const model);
  void SetPoints(std::list<PointSet>* const points);
  itkSetMacro(NumberOfParameters, unsigned int);

  virtual unsigned int GetNumberOfValues(void) const ITK_OVERRIDE;
  virtual unsigned int GetNumberOfParameters() const ITK_OVERRIDE;
  virtual MeasureType GetValue( const ParametersType & parameters ) const ITK_OVERRIDE;
  virtual void GetDerivative( const ParametersType & parameters, DerivativeType  & derivative ) const ITK_OVERRIDE;
  double GetRMS(const MeasureType& values) const;

protected:

  NonLinearCostFunction();
  virtual ~NonLinearCostFunction();

  NonLinearCostFunction(const NonLinearCostFunction&); // Purposefully not implemented.
  NonLinearCostFunction& operator=(const NonLinearCostFunction&); // Purposefully not implemented.

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const = 0;

  Model3D              *m_Model;
  std::list<PointSet>  *m_Points;
  unsigned int          m_NumberOfParameters;
  unsigned int          m_NumberOfValues;
};

} // end namespace

#endif
