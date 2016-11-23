/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoCalibrationCostFunction_h
#define niftkNonLinearStereoCalibrationCostFunction_h

#include <niftkNiftyCalTypes.h>
#include "niftkNonLinearCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoCalibrationCostFunction
* \brief Computes cost as RMS reconstruction (3D) error
* for intrinsic, extrinsic and stereo optimisation.
*
* \see niftk::NonLinearStereoCalibrationOptimiser
*/
class NonLinearStereoCalibrationCostFunction :
    public niftk::NonLinearCostFunction
{

public:

  typedef NonLinearStereoCalibrationCostFunction Self;
  typedef niftk::NonLinearCostFunction           Superclass;
  typedef itk::SmartPointer<Self>                Pointer;
  typedef itk::SmartPointer<const Self>          ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType             ParametersType;
  typedef Superclass::DerivativeType             DerivativeType;
  typedef Superclass::MeasureType                MeasureType;

  void SetRightHandPoints(std::list<PointSet>* const points);
  virtual unsigned int GetNumberOfValues(void) const ITK_OVERRIDE;
  itkSetMacro(NumberOfValues, unsigned int);

  virtual MeasureType InternalGetValue( const ParametersType & parameters ) const ITK_OVERRIDE;

protected:

  NonLinearStereoCalibrationCostFunction();
  virtual ~NonLinearStereoCalibrationCostFunction();

  NonLinearStereoCalibrationCostFunction(const NonLinearStereoCalibrationCostFunction&);
  NonLinearStereoCalibrationCostFunction& operator=(const NonLinearStereoCalibrationCostFunction&);

  std::list<PointSet> *m_RightHandPoints;
};

} // end namespace

#endif
