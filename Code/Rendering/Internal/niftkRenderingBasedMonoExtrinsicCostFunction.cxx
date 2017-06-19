/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingBasedMonoExtrinsicCostFunction.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>

namespace niftk
{

//-----------------------------------------------------------------------------
RenderingBasedMonoExtrinsicCostFunction::RenderingBasedMonoExtrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
RenderingBasedMonoExtrinsicCostFunction::~RenderingBasedMonoExtrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
void RenderingBasedMonoExtrinsicCostFunction::Initialise(vtkRenderWindow* win,
                                                         const cv::Size2i& windowSize,
                                                         const cv::Size2i& calibratedWindowSize,
                                                         const std::string& model,
                                                         const std::string& texture,
                                                         const std::vector<cv::Mat>& videoImages,
                                                         const cv::Mat& intrinsics,
                                                         const cv::Mat& distortion
                                                        )
{
  Superclass::Initialise(win, windowSize, calibratedWindowSize, model, texture, videoImages);

  m_Intrinsics = intrinsics;
  m_Distortion = distortion;

  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::undistort(m_OriginalVideoImagesInGreyScale[i],
                  m_UndistortedVideoImagesInGreyScale[i],
                  m_Intrinsics, m_Distortion, m_Intrinsics);
  }
}


//-----------------------------------------------------------------------------
unsigned int RenderingBasedMonoExtrinsicCostFunction::GetNumberOfParameters(void) const
{
  return m_OriginalVideoImages.size() * 6;
}


//-----------------------------------------------------------------------------
RenderingBasedMonoExtrinsicCostFunction::MeasureType
RenderingBasedMonoExtrinsicCostFunction::GetValue(const ParametersType & parameters) const
{  
  MeasureType cost = 0;
  cv::Mat jointHist = cv::Mat::zeros(16, 16, CV_64FC1);
  cv::Mat histogramRows = cv::Mat::zeros(16, 1, CV_64FC1);
  cv::Mat histogramCols = cv::Mat::zeros(1, 16, CV_64FC1);
  unsigned long int counter = 0;

  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::Mat rotation = cv::Mat::zeros(1, 3, CV_64FC1);
    rotation.at<double>(0, 0) = parameters[i*6 + 0];
    rotation.at<double>(0, 1) = parameters[i*6 + 1];
    rotation.at<double>(0, 2) = parameters[i*6 + 2];

    cv::Mat translation = cv::Mat::zeros(1, 3, CV_64FC1);
    translation.at<double>(0, 0) = parameters[i*6 + 3];
    translation.at<double>(0, 1) = parameters[i*6 + 4];
    translation.at<double>(0, 2) = parameters[i*6 + 5];

    cv::Matx44d worldToCamera = niftk::RodriguesToMatrix(rotation, translation);
    m_Pipeline->SetWorldToCameraMatrix(worldToCamera);
    m_Pipeline->SetIntrinsics(m_Intrinsics);
    m_Pipeline->CopyScreen(m_RenderedImage);
    cv::cvtColor(m_RenderedImage, m_RenderedImageInGreyscale, CV_BGR2GRAY);

    this->AccumulateSamples(m_UndistortedVideoImagesInGreyScale[i],
                            counter, histogramRows, histogramCols, jointHist);

  }

  cost = this->ComputeNMI(counter, histogramRows, histogramCols, jointHist);

  return cost;
}


//-----------------------------------------------------------------------------
RenderingBasedMonoExtrinsicCostFunction::ParametersType
RenderingBasedMonoExtrinsicCostFunction::GetStepSizes() const
{
  ParametersType stepSize;
  stepSize.SetSize(this->GetNumberOfParameters());

  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    stepSize[i*6 + 0] = 0.01; // r1 (Rodrigues)
    stepSize[i*6 + 1] = 0.01; // r2 (Rodrigues)
    stepSize[i*6 + 2] = 0.01; // r3 (Rodrigues)
    stepSize[i*6 + 3] = 0.5;  // tx
    stepSize[i*6 + 4] = 0.5;  // ty
    stepSize[i*6 + 5] = 0.5;  // tz
  }
  return stepSize;
}

} // end niftk
