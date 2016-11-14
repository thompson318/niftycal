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

#include <niftkWin32ExportHeader.h>
#include "niftkNonLinearCostFunction.h"

namespace niftk
{

/**
* \class NonLinearHandEyeCostFunction
* \brief Base class for non-linear hand-eye cost functions.
*/
class NIFTYCAL_WINEXPORT NonLinearHandEyeCostFunction :
    public niftk::NonLinearCostFunction
{

public:

  typedef NonLinearHandEyeCostFunction   Self;
  typedef niftk::NonLinearCostFunction   Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;

  typedef Superclass::ParametersType     ParametersType;
  typedef Superclass::DerivativeType     DerivativeType;
  typedef Superclass::MeasureType        MeasureType;

  void SetHandMatrices(std::list<cv::Matx44d>* const matrices);
  std::list<cv::Matx44d>* GetHandMatrices() const;
  virtual MeasureType GetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearHandEyeCostFunction();
  virtual ~NonLinearHandEyeCostFunction();

  NonLinearHandEyeCostFunction(const NonLinearHandEyeCostFunction&); // Purposefully not implemented.
  NonLinearHandEyeCostFunction& operator=(const NonLinearHandEyeCostFunction&); // Purposefully not implemented.

  std::list<cv::Matx44d> *m_HandMatrices;
};

} // end namespace

#endif
