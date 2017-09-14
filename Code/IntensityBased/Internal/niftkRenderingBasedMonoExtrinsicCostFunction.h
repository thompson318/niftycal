/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkRenderingBasedMonoExtrinsicCostFunction_h
#define niftkRenderingBasedMonoExtrinsicCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkRenderingBasedCostFunction.h"

namespace niftk
{

/**
 * \class RenderingBasedMonoExtrinsicCostFunction
 * \brief Optimises Extrinsic parameters only by matching to rendering of model.
 */
class RenderingBasedMonoExtrinsicCostFunction : public niftk::RenderingBasedCostFunction
{

public:

  typedef RenderingBasedMonoExtrinsicCostFunction Self;
  typedef niftk::RenderingBasedCostFunction       Superclass;
  typedef itk::SmartPointer<Self>                 Pointer;
  typedef itk::SmartPointer<const Self>           ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType              ParametersType;
  typedef Superclass::DerivativeType              DerivativeType;
  typedef Superclass::MeasureType                 MeasureType;

  /**
   * \see itk::CostFunction::GetNumberOfParameters()
   */
  virtual unsigned int GetNumberOfParameters(void) const;

  /**
   * \see itk::SingleValuedCostFunction::GetValue()
   */
  virtual MeasureType GetValue(const ParametersType & parameters) const;

  /**
   * \brief Instantiates the internal rendering pipeline.
   */
  void Initialise(vtkRenderWindow* win,
                  const cv::Size2i& windowSize,
                  const cv::Size2i& calibratedWindowSize,
                  const std::string& model,
                  const std::string& texture,
                  const std::vector<cv::Mat>& videoImages,
                  const cv::Mat& intrinsics,
                  const cv::Mat& distortion
                 );

protected:

  RenderingBasedMonoExtrinsicCostFunction(); // deliberately protected.
  virtual ~RenderingBasedMonoExtrinsicCostFunction(); // deliberately protected.

  RenderingBasedMonoExtrinsicCostFunction(const RenderingBasedMonoExtrinsicCostFunction&); // deliberately not implemented
  RenderingBasedMonoExtrinsicCostFunction& operator=(const RenderingBasedMonoExtrinsicCostFunction&); // deliberately not implemented

  virtual ParametersType GetStepSizes() const;

private:

  cv::Mat m_Intrinsics;
  cv::Mat m_Distortion;

};

} // end namespace

#endif


