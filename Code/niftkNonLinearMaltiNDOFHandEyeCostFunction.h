/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMaltiNDOFHandEyeCostFunction_h
#define niftkNonLinearMaltiNDOFHandEyeCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkNonLinearHandEyeCostFunction.h"

namespace niftk
{

/**
* \class NonLinearMaltiNDOFHandEyeCostFunction
* \brief Computes cost as RMS re-projection error for extrinsic,
* hand-eye and model-to-world optimisation, as an extension of
* <a href="http://dx.doi.org/10.1002/rcs.1478">Malti 2013</a>.
*
* \see niftk::NonLinearMaltiNDOFHandEyeOptimiser
*/
class NIFTYCAL_WINEXPORT NonLinearMaltiNDOFHandEyeCostFunction :
    public niftk::NonLinearHandEyeCostFunction
{

public:

  typedef NonLinearMaltiNDOFHandEyeCostFunction Self;
  typedef NonLinearHandEyeCostFunction          Superclass;
  typedef itk::SmartPointer<Self>               Pointer;
  typedef itk::SmartPointer<const Self>         ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType            ParametersType;
  typedef Superclass::DerivativeType            DerivativeType;
  typedef Superclass::MeasureType               MeasureType;

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;
  void SetIntrinsic(cv::Mat* const intrinsic);
  void SetDistortion(cv::Mat* const distortion);

protected:

  NonLinearMaltiNDOFHandEyeCostFunction();
  virtual ~NonLinearMaltiNDOFHandEyeCostFunction();

  NonLinearMaltiNDOFHandEyeCostFunction(const NonLinearMaltiNDOFHandEyeCostFunction&);
  NonLinearMaltiNDOFHandEyeCostFunction& operator=(const NonLinearMaltiNDOFHandEyeCostFunction&);

private:

  cv::Mat* m_Intrinsic;
  cv::Mat* m_Distortion;
};

} // end namespace

#endif
