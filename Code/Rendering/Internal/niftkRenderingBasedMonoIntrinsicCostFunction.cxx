/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingBasedMonoIntrinsicCostFunction.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>

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
  if (rvecs.size() != videoImages.size())
  {
    niftkNiftyCalThrow() << "Number of rotation vectors (" << rvecs.size()
                         << "), doesn't match number of images (" << videoImages.size() << ")";
  }

  if (tvecs.size() != videoImages.size())
  {
    niftkNiftyCalThrow() << "Number of translation vectors (" << tvecs.size()
                         << "), doesn't match number of images (" << videoImages.size() << ")";
  }

  m_Pipeline.reset(new CalibratedRenderingPipeline(windowSize,
                                                   calibratedWindowSize,
                                                   model,
                                                   texture,
                                                   true));

  m_Pipeline->SetIntrinsics(intrinsic); // so these rendered images are fixed during optimisation.

  m_OriginalVideoImages = videoImages;
  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::Mat video;
    m_OriginalVideoImages[i].copyTo(video);
    m_UndistortedVideoImages.push_back(video);

    cv::Mat rendering;
    m_OriginalVideoImages[i].copyTo(rendering);
    m_RenderedImages.push_back(rendering);

    cv::Matx44d worldToCamera = niftk::RodriguesToMatrix(rvecs[i], tvecs[i]);
    m_Pipeline->SetWorldToCameraMatrix(worldToCamera);

    m_Pipeline->DumpScreen(m_RenderedImages[i]);
  }
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
