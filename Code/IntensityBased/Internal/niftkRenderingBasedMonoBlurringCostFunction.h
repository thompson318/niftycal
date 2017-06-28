/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkRenderingBasedMonoBlurringCostFunction_h
#define niftkRenderingBasedMonoBlurringCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkRenderingBasedCostFunction.h"

namespace niftk
{

/**
 * \class RenderingBasedMonoBlurringCostFunction
 * \brief Optimises blurring parameters only by matching to rendering of model.
 */
class RenderingBasedMonoBlurringCostFunction : public niftk::RenderingBasedCostFunction
{

public:

  typedef RenderingBasedMonoBlurringCostFunction Self;
  typedef niftk::RenderingBasedCostFunction      Superclass;
  typedef itk::SmartPointer<Self>                Pointer;
  typedef itk::SmartPointer<const Self>          ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType             ParametersType;
  typedef Superclass::DerivativeType             DerivativeType;
  typedef Superclass::MeasureType                MeasureType;

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
                  const cv::Mat& distortion,
                  const std::vector<cv::Mat>& rvecs,
                  const std::vector<cv::Mat>& tvecs
                 );

protected:

  RenderingBasedMonoBlurringCostFunction(); // deliberately protected.
  virtual ~RenderingBasedMonoBlurringCostFunction(); // deliberately protected.

  RenderingBasedMonoBlurringCostFunction(const RenderingBasedMonoBlurringCostFunction&); // deliberately not implemented
  RenderingBasedMonoBlurringCostFunction& operator=(const RenderingBasedMonoBlurringCostFunction&); // deliberately not implemented

  virtual ParametersType GetStepSizes() const;

private:

  cv::Mat m_Intrinsics;
  cv::Mat m_Distortion;
  std::vector<cv::Mat> m_Rvecs;
  std::vector<cv::Mat> m_Tvecs;
};

} // end namespace

#endif


