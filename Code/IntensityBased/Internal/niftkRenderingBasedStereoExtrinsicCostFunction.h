/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkRenderingBasedStereoExtrinsicCostFunction_h
#define niftkRenderingBasedStereoExtrinsicCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkRenderingBasedCostFunction.h"

namespace niftk
{

/**
 * \class RenderingBasedStereoExtrinsicCostFunction
 * \brief Optimises Extrinsic parameters only by matching to rendering of model.
 */
class RenderingBasedStereoExtrinsicCostFunction : public niftk::RenderingBasedCostFunction
{

public:

  typedef RenderingBasedStereoExtrinsicCostFunction Self;
  typedef niftk::RenderingBasedCostFunction         Superclass;
  typedef itk::SmartPointer<Self>                   Pointer;
  typedef itk::SmartPointer<const Self>             ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType                ParametersType;
  typedef Superclass::DerivativeType                DerivativeType;
  typedef Superclass::MeasureType                   MeasureType;

  itkSetMacro(SigmaRight, double);
  itkGetMacro(SigmaRight, double);

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
                  const std::vector<cv::Mat>& leftVideoImages,
                  const std::vector<cv::Mat>& rightVideoImages,
                  const cv::Mat& leftIntrinsics,
                  const cv::Mat& leftDistortion,
                  const cv::Mat& rightIntrinsics,
                  const cv::Mat& rightDistortion,
                  const cv::Mat& leftToRightRotationMatrix,
                  const cv::Mat& leftToRightTranslationVector
                 );

protected:

  RenderingBasedStereoExtrinsicCostFunction(); // deliberately protected.
  virtual ~RenderingBasedStereoExtrinsicCostFunction(); // deliberately protected.

  RenderingBasedStereoExtrinsicCostFunction(const RenderingBasedStereoExtrinsicCostFunction&); // deliberately not implemented
  RenderingBasedStereoExtrinsicCostFunction& operator=(const RenderingBasedStereoExtrinsicCostFunction&); // deliberately not implemented

  virtual ParametersType GetStepSizes() const;

private:

  cv::Mat                      m_LeftIntrinsics;
  cv::Mat                      m_LeftDistortion;
  cv::Mat                      m_RightIntrinsics;
  cv::Mat                      m_RightDistortion;
  cv::Mat                      m_LeftToRightRVec;
  cv::Mat                      m_LeftToRightTVec;
  std::vector<cv::Mat>         m_RightOriginalVideoImages;
  std::vector<cv::Mat>         m_RightOriginalVideoImagesInGreyScale;
  mutable std::vector<cv::Mat> m_RightUndistortedVideoImagesInGreyScale;
  double                       m_SigmaRight;

};

} // end namespace

#endif


