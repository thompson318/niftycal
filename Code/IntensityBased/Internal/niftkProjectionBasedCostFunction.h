/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkProjectionBasedCostFunction_h
#define niftkProjectionBasedCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkIntensityBasedCostFunction.h"
#include <vector>

namespace niftk
{

/**
 * \class ProjectionBasedCostFunction
 * \brief Base class for cost functions that measure cost by projecting 3D points to 2D.
 */
class ProjectionBasedCostFunction : public niftk::IntensityBasedCostFunction
{

public:

  typedef ProjectionBasedCostFunction       Self;
  typedef niftk::IntensityBasedCostFunction Superclass;
  typedef itk::SmartPointer<Self>           Pointer;
  typedef itk::SmartPointer<const Self>     ConstPointer;

  typedef Superclass::ParametersType        ParametersType;
  typedef Superclass::DerivativeType        DerivativeType;
  typedef Superclass::MeasureType           MeasureType;

  /**
   * \see itk::CostFunction::GetNumberOfParameters()
   */
  virtual unsigned int GetNumberOfParameters(void) const = 0;

  /**
   * \see itk::SingleValuedCostFunction::GetValue()
   */
  virtual MeasureType GetValue(const ParametersType & parameters) const = 0;

  /**
   * \brief Instantiates the internal list of points.
   */
  void Initialise(const cv::Size2i& windowSize,
                  const std::string& model,
                  const unsigned int& histogramDivisor
                 );

  virtual void SetActivated(const bool& isActivated) {}

protected:

  ProjectionBasedCostFunction(); // deliberately protected.
  virtual ~ProjectionBasedCostFunction(); // deliberately protected.

  ProjectionBasedCostFunction(const ProjectionBasedCostFunction&); // deliberately not implemented
  ProjectionBasedCostFunction& operator=(const ProjectionBasedCostFunction&); // deliberately not implemented

  void AccumulateSamples(const cv::Mat& greyScaleVideoImageA,
                         const cv::Mat& intrinsicsA,
                         const cv::Mat& distortionA,
                         const cv::Mat& rvecA,
                         const cv::Mat& tvecA,
                         const cv::Mat& greyScaleVideoImageB,
                         const cv::Mat& intrinsicsB,
                         const cv::Mat& distortionB,
                         const cv::Mat& rvecB,
                         const cv::Mat& tvecB,
                         unsigned long int& counter,
                         cv::Mat& histogramRows,
                         cv::Mat& histogramCols,
                         cv::Mat& jointHistogram
                        ) const;

  virtual ParametersType GetStepSizes() const = 0;

  unsigned int BiLinearInterpolate(const cv::Mat& image, cv::Point2f& pixel) const;

  cv::Size2i               m_WindowSize;
  std::vector<cv::Point3f> m_Model;
  unsigned int             m_HistogramDivisor;
};

} // end namespace

#endif


