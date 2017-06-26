/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkProjectionBasedMonoIntrinsicCostFunction_h
#define niftkProjectionBasedMonoIntrinsicCostFunction_h

#include "niftkWin32ExportHeader.h"
#include "niftkProjectionBasedCostFunction.h"
#include <vector>

namespace niftk
{

/**
 * \class ProjectionBasedMonoIntrinsicCostFunction
 * \brief Computes cost, based on NMI of pixels that match across a set of views.
 */
class ProjectionBasedMonoIntrinsicCostFunction : public niftk::ProjectionBasedCostFunction
{

public:

  typedef ProjectionBasedMonoIntrinsicCostFunction Self;
  typedef niftk::ProjectionBasedCostFunction       Superclass;
  typedef itk::SmartPointer<Self>                  Pointer;
  typedef itk::SmartPointer<const Self>            ConstPointer;
  itkNewMacro(Self);

  typedef Superclass::ParametersType               ParametersType;
  typedef Superclass::DerivativeType               DerivativeType;
  typedef Superclass::MeasureType                  MeasureType;

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
                  const std::vector<cv::Mat>& videoImages,
                  const std::vector<cv::Mat>& rvecs,
                  const std::vector<cv::Mat>& tvecs
                 );

protected:

  ProjectionBasedMonoIntrinsicCostFunction(); // deliberately protected.
  virtual ~ProjectionBasedMonoIntrinsicCostFunction(); // deliberately protected.

  ProjectionBasedMonoIntrinsicCostFunction(const ProjectionBasedMonoIntrinsicCostFunction&); // deliberately not implemented
  ProjectionBasedMonoIntrinsicCostFunction& operator=(const ProjectionBasedMonoIntrinsicCostFunction&); // deliberately not implemented

  virtual ParametersType GetStepSizes() const;

  std::vector<cv::Mat> m_Rvecs;
  std::vector<cv::Mat> m_Tvecs;
  std::vector<cv::Mat> m_Images;
  std::vector<cv::Mat> m_ImagesInGreyScale;
};

} // end namespace

#endif


