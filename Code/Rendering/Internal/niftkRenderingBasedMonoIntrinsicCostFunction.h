/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkRenderingBasedMonoIntrinsicCostFunction_h
#define niftkRenderingBasedMonoIntrinsicCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkCalibratedRenderingPipeline.h"
#include <itkSingleValuedCostFunction.h>
#include <cv.h>
#include <vector>
#include <memory>

namespace niftk
{

/**
 * \class RenderingBasedMonoIntrinsicCostFunction
 * \brief Optimises intrinsic parameters only by matching to rendering of model.
 */
class RenderingBasedMonoIntrinsicCostFunction : public itk::SingleValuedCostFunction
{

public:

  typedef RenderingBasedMonoIntrinsicCostFunction Self;
  typedef itk::SingleValuedCostFunction           Superclass;
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
   * \see itk::SingleValuedCostFunction::GetDerivative()
   */
  virtual void GetDerivative(const ParametersType & parameters,
                             DerivativeType & derivative) const;

  /**
   * \brief Instantiates the internal rendering pipeline.
   */
  void Initialise(const cv::Size2i& windowSize,
                  const cv::Size2i& calibratedWindowSize,
                  const std::string& model,
                  const std::string& texture,
                  const std::vector<cv::Mat>& videoImages,
                  const std::vector<cv::Mat>& rvecs,
                  const std::vector<cv::Mat>& tvecs,
                  const cv::Mat& intrinsic,
                  const cv::Mat& distortion
                 );

protected:

  RenderingBasedMonoIntrinsicCostFunction(); // deliberately protected.
  virtual ~RenderingBasedMonoIntrinsicCostFunction(); // deliberately protected.

  RenderingBasedMonoIntrinsicCostFunction(const RenderingBasedMonoIntrinsicCostFunction&); // deliberately not implemented
  RenderingBasedMonoIntrinsicCostFunction& operator=(const RenderingBasedMonoIntrinsicCostFunction&); // deliberately not implemented

private:

  std::vector<cv::Mat>                         m_OriginalVideoImages;
  std::vector<cv::Mat>                         m_OriginalVideoImagesInGreyScale;
  std::vector<cv::Mat>                         m_UndistortedVideoImagesInGreyScale;
  std::vector<cv::Mat>                         m_RenderedImages;
  std::vector<cv::Mat>                         m_RenderedImagesInGreyscale;
  std::unique_ptr<CalibratedRenderingPipeline> m_Pipeline;
};

} // end namespace

#endif


