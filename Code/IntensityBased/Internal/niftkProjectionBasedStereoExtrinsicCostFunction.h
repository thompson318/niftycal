/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkProjectionBasedStereoExtrinsicCostFunction_h
#define niftkProjectionBasedStereoExtrinsicCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkProjectionBasedCostFunction.h"
#include <vector>

namespace niftk
{

/**
 * \class ProjectionBasedStereoExtrinsicCostFunction
 * \brief Computes cost, based on NMI of pixels that match across stereo views.
 */
class ProjectionBasedStereoExtrinsicCostFunction : public niftk::ProjectionBasedCostFunction
{

public:

  typedef ProjectionBasedStereoExtrinsicCostFunction Self;
  typedef niftk::ProjectionBasedCostFunction         Superclass;
  typedef itk::SmartPointer<Self>                    Pointer;
  typedef itk::SmartPointer<const Self>              ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType                 ParametersType;
  typedef Superclass::DerivativeType                 DerivativeType;
  typedef Superclass::MeasureType                    MeasureType;

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
  void Initialise(const cv::Size2i& windowSize,
                  const std::string& model,
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

  ProjectionBasedStereoExtrinsicCostFunction(); // deliberately protected.
  virtual ~ProjectionBasedStereoExtrinsicCostFunction(); // deliberately protected.

  ProjectionBasedStereoExtrinsicCostFunction(const ProjectionBasedStereoExtrinsicCostFunction&); // deliberately not implemented
  ProjectionBasedStereoExtrinsicCostFunction& operator=(const ProjectionBasedStereoExtrinsicCostFunction&); // deliberately not implemented

  virtual ParametersType GetStepSizes() const;

  cv::Mat              m_LeftIntrinsics;
  cv::Mat              m_LeftDistortion;
  cv::Mat              m_RightIntrinsics;
  cv::Mat              m_RightDistortion;
  cv::Mat              m_LeftToRightRVec;
  cv::Mat              m_LeftToRightTVec;
  std::vector<cv::Mat> m_LeftImages;
  std::vector<cv::Mat> m_RightImages;
};

} // end namespace

#endif


