/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkRenderingBasedCostFunction_h
#define niftkRenderingBasedCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkCalibratedRenderingPipeline.h"
#include "niftkIntensityBasedCostFunction.h"
#include <vector>
#include <memory>

namespace niftk
{

/**
 * \class RenderingBasedCostFunction
 * \brief Base class for cost functions that measure cost by matching to rendering of model.
 */
class RenderingBasedCostFunction : public niftk::IntensityBasedCostFunction
{

public:

  typedef RenderingBasedCostFunction        Self;
  typedef niftk::IntensityBasedCostFunction Superclass;
  typedef itk::SmartPointer<Self>           Pointer;
  typedef itk::SmartPointer<const Self>     ConstPointer;

  typedef Superclass::ParametersType        ParametersType;
  typedef Superclass::DerivativeType        DerivativeType;
  typedef Superclass::MeasureType           MeasureType;

  itkSetMacro(Sigma, double);
  itkGetMacro(Sigma, double);
  itkSetMacro(UseBlurring, bool);
  itkGetMacro(UseBlurring, bool);

  /**
   * \see itk::CostFunction::GetNumberOfParameters()
   */
  virtual unsigned int GetNumberOfParameters(void) const = 0;

  /**
   * \see itk::SingleValuedCostFunction::GetValue()
   */
  virtual MeasureType GetValue(const ParametersType & parameters) const = 0;

  /**
   * \brief Instantiates the internal rendering pipeline.
   */
  void Initialise(vtkRenderWindow* win,
                  const cv::Size2i& windowSize,
                  const cv::Size2i& calibratedWindowSize,
                  const std::string& model,
                  const std::string& texture,
                  const std::vector<cv::Mat>& videoImages
                 );

  virtual void SetActivated(const bool& isActivated);

protected:

  RenderingBasedCostFunction(); // deliberately protected.
  virtual ~RenderingBasedCostFunction(); // deliberately protected.

  RenderingBasedCostFunction(const RenderingBasedCostFunction&); // deliberately not implemented
  RenderingBasedCostFunction& operator=(const RenderingBasedCostFunction&); // deliberately not implemented

  void AccumulateSamples(const cv::Mat& greyScaleVideoImage,
                         const double& sigma,
                         unsigned long int& counter,
                         cv::Mat& histogramRows,
                         cv::Mat& histogramCols,
                         cv::Mat& jointHistogram
                        ) const;

  virtual ParametersType GetStepSizes() const = 0;

  std::vector<cv::Mat>                         m_OriginalVideoImages;
  std::vector<cv::Mat>                         m_OriginalVideoImagesInGreyScale;
  mutable std::vector<cv::Mat>                 m_UndistortedVideoImagesInGreyScale;
  mutable cv::Mat                              m_RenderedImage;
  mutable cv::Mat                              m_RenderedImageInGreyscale;
  double                                       m_Sigma;
  bool                                         m_UseBlurring;
  mutable cv::Mat                              m_RenderedImageBlurred;
  std::unique_ptr<CalibratedRenderingPipeline> m_Pipeline;
  cv::Vec3b                                    m_BackgroundColour; // (BGR, pure Blue)
};

} // end namespace

#endif


