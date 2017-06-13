/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingBasedMonoIntrinsicCostFunction.h"

namespace niftk
{

//-----------------------------------------------------------------------------
RenderingBasedMonoIntrinsicCostFunction::RenderingBasedMonoIntrinsicCostFunction()
: m_Pipeline(nullptr)
{
}


//-----------------------------------------------------------------------------
RenderingBasedMonoIntrinsicCostFunction::~RenderingBasedMonoIntrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
void RenderingBasedMonoIntrinsicCostFunction::Initialise(const cv::Size2i& windowSize,
                                                         const cv::Size2i& calibratedWindowSize,
                                                         const std::string& model,
                                                         const std::string& texture,
                                                         const std::vector<cv::Mat>& videoImages,
                                                         const std::vector<cv::Mat>& rvecs,
                                                         const std::vector<cv::Mat>& tvecs,
                                                         const cv::Mat& intrinsic,
                                                         const cv::Mat& distortion
                                                        )
{
}


//-----------------------------------------------------------------------------
unsigned int RenderingBasedMonoIntrinsicCostFunction::GetNumberOfParameters(void) const
{
  return 8;
}


//-----------------------------------------------------------------------------
RenderingBasedMonoIntrinsicCostFunction::MeasureType
RenderingBasedMonoIntrinsicCostFunction::GetValue(const ParametersType & parameters) const
{
  return 0;
}


//-----------------------------------------------------------------------------
void RenderingBasedMonoIntrinsicCostFunction::GetDerivative(const ParametersType & parameters,
                                                            DerivativeType & derivative) const
{
}

} // end niftk
