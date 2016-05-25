/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearHandEyeCostFunction_h
#define niftkNonLinearHandEyeCostFunction_h

#include "niftkWin32ExportHeader.h"
#include <cv.h>
#include <itkMultipleValuedCostFunction.h>
#include <niftkNiftyCalTypes.h>

namespace niftk
{

/**
* \brief Base class for non-linear hand-eye cost functions.
*/
class NIFTYCAL_WINEXPORT NonLinearHandEyeCostFunction :
    public itk::MultipleValuedCostFunction
{

public:

  typedef NonLinearHandEyeCostFunction          Self;
  typedef itk::MultipleValuedCostFunction       Superclass;
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;

  typedef Superclass::ParametersType            ParametersType;
  typedef Superclass::DerivativeType            DerivativeType;
  typedef Superclass::MeasureType               MeasureType;

  void SetModel(Model3D* const model);
  void SetPoints(std::list<PointSet>* const points);
  void SetHandMatrices(std::list<cv::Matx44d>* const matrices);
  std::list<cv::Matx44d>* GetHandMatrices() const;
  itkSetMacro(NumberOfParameters, unsigned int);

  virtual unsigned int GetNumberOfValues(void) const ITK_OVERRIDE;
  virtual unsigned int GetNumberOfParameters() const ITK_OVERRIDE;
  virtual MeasureType GetValue( const ParametersType & parameters ) const ITK_OVERRIDE;
  virtual void GetDerivative( const ParametersType & parameters, DerivativeType  & derivative ) const ITK_OVERRIDE;
  double GetRMS(const MeasureType& values) const;

protected:

  NonLinearHandEyeCostFunction();
  virtual ~NonLinearHandEyeCostFunction();

  NonLinearHandEyeCostFunction(const NonLinearHandEyeCostFunction&); // Purposefully not implemented.
  NonLinearHandEyeCostFunction& operator=(const NonLinearHandEyeCostFunction&); // Purposefully not implemented.

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const = 0;

  Model3D                *m_Model;
  std::list<PointSet>    *m_Points;
  std::list<cv::Matx44d> *m_HandMatrices;
  unsigned int            m_NumberOfParameters;
  unsigned int            m_NumberOfValues;
};

} // end namespace

#endif