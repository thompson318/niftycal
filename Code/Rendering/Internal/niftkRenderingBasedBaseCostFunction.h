/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkRenderingBasedBaseCostFunction_h
#define niftkRenderingBasedBaseCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkCalibratedRenderingPipeline.h"
#include <itkSingleValuedCostFunction.h>
#include <cv.h>
#include <vector>
#include <memory>

namespace niftk
{

/**
 * \class RenderingBasedBaseCostFunction
 * \brief Base class for cost functions that measure cost by matching to rendering of model.
 */
class RenderingBasedBaseCostFunction : public itk::SingleValuedCostFunction
{

public:

  typedef RenderingBasedBaseCostFunction Self;
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
   * \brief Instantiates the internal rendering pipeline.
   */
  void Initialise(vtkRenderWindow* win,
                  const cv::Size2i& windowSize,
                  const cv::Size2i& calibratedWindowSize,
                  const std::string& model,
                  const std::string& texture,
                  const std::vector<cv::Mat>& videoImages
                 );

protected:

  RenderingBasedBaseCostFunction(); // deliberately protected.
  virtual ~RenderingBasedBaseCostFunction(); // deliberately protected.

  RenderingBasedBaseCostFunction(const RenderingBasedBaseCostFunction&); // deliberately not implemented
  RenderingBasedBaseCostFunction& operator=(const RenderingBasedBaseCostFunction&); // deliberately not implemented

  double ComputeNMI(const unsigned long int& counter,
                    const cv::Mat& histogramRows,
                    const cv::Mat& histogramCols,
                    const cv::Mat& jointHist) const;

  void AccumulateSamples(unsigned long int& counter,
                         cv::Mat& histogramRows,
                         cv::Mat& histogramCols,
                         cv::Mat& jointHistogram
                        ) const;

  virtual ParametersType GetStepSizes() const = 0;

  std::vector<cv::Mat>                         m_OriginalVideoImages;
  std::vector<cv::Mat>                         m_OriginalVideoImagesInGreyScale;
  mutable std::vector<cv::Mat>                 m_UndistortedVideoImagesInGreyScale;
  mutable std::vector<cv::Mat>                 m_RenderedImages;
  mutable std::vector<cv::Mat>                 m_RenderedImagesInGreyscale;
  std::unique_ptr<CalibratedRenderingPipeline> m_Pipeline;
  cv::Vec3i                                    m_BackgroundColour; // (BGR, pure Blue)

};

} // end namespace

#endif


