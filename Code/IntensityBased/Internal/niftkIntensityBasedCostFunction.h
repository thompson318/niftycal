/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkIntensityBasedCostFunction_h
#define niftkIntensityBasedCostFunction_h

#include "niftkWin32ExportHeader.h"
#include <itkSingleValuedCostFunction.h>
#include <cv.h>
#include <vector>

namespace niftk
{

/**
 * \class IntensityBasedCostFunction
 * \brief Base class for cost functions that measure cost based on NMI of intensities.
 */
class IntensityBasedCostFunction : public itk::SingleValuedCostFunction
{

public:

  typedef IntensityBasedCostFunction     Self;
  typedef itk::SingleValuedCostFunction  Superclass;
  typedef itk::SmartPointer<Self>        Pointer;
  typedef itk::SmartPointer<const Self>  ConstPointer;

  typedef Superclass::ParametersType     ParametersType;
  typedef Superclass::DerivativeType     DerivativeType;
  typedef Superclass::MeasureType        MeasureType;

  /**
   * \see itk::CostFunction::GetNumberOfParameters()
   */
  virtual unsigned int GetNumberOfParameters(void) const = 0;

  /**
   * \see itk::SingleValuedCostFunction::GetValue()
   */
  virtual MeasureType GetValue(const ParametersType & parameters) const = 0;

  /**
   * \see itk::SingleValuedCostFunction::GetDerivative()
   */
  virtual void GetDerivative(const ParametersType & parameters,
                             DerivativeType & derivative) const;

  /**
   * \brief Used to tell base classes to enter Activated/Not Activated state.
   */
  virtual void SetActivated(const bool& isActivated) = 0;

protected:

  IntensityBasedCostFunction(); // deliberately protected.
  virtual ~IntensityBasedCostFunction(); // deliberately protected.

  IntensityBasedCostFunction(const IntensityBasedCostFunction&); // deliberately not implemented
  IntensityBasedCostFunction& operator=(const IntensityBasedCostFunction&); // deliberately not implemented

  double ComputeNMI(const unsigned long int& counter,
                    const cv::Mat& histogramRows,
                    const cv::Mat& histogramCols,
                    const cv::Mat& jointHist) const;

  virtual ParametersType GetStepSizes() const = 0; // used for derivative

};

} // end namespace

#endif


