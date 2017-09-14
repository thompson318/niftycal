/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingBasedMonoBlurringCostFunction.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>

namespace niftk
{

//-----------------------------------------------------------------------------
RenderingBasedMonoBlurringCostFunction::RenderingBasedMonoBlurringCostFunction()
{
}


//-----------------------------------------------------------------------------
RenderingBasedMonoBlurringCostFunction::~RenderingBasedMonoBlurringCostFunction()
{
}


//-----------------------------------------------------------------------------
void RenderingBasedMonoBlurringCostFunction::Initialise(vtkRenderWindow* win,
                                                        const cv::Size2i& windowSize,
                                                        const cv::Size2i& calibratedWindowSize,
                                                        const std::string& model,
                                                        const std::string& texture,
                                                        const std::vector<cv::Mat>& videoImages,
                                                        const cv::Mat& intrinsics,
                                                        const cv::Mat& distortion,
                                                        const std::vector<cv::Mat>& rvecs,
                                                        const std::vector<cv::Mat>& tvecs
                                                       )
{
  Superclass::Initialise(win, windowSize, calibratedWindowSize, model, texture, videoImages);

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

  m_Intrinsics = intrinsics;
  m_Distortion = distortion;
  m_Rvecs = rvecs;
  m_Tvecs = tvecs;

  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::undistort(m_OriginalVideoImagesInGreyScale[i],
                  m_UndistortedVideoImagesInGreyScale[i],
                  m_Intrinsics, m_Distortion, m_Intrinsics);
  }
}


//-----------------------------------------------------------------------------
unsigned int RenderingBasedMonoBlurringCostFunction::GetNumberOfParameters(void) const
{
  return 1;
}


//-----------------------------------------------------------------------------
RenderingBasedMonoBlurringCostFunction::MeasureType
RenderingBasedMonoBlurringCostFunction::GetValue(const ParametersType & parameters) const
{  
  MeasureType cost = 0;
  cv::Mat jointHist = cv::Mat::zeros(16, 16, CV_64FC1);
  cv::Mat histogramRows = cv::Mat::zeros(16, 1, CV_64FC1);
  cv::Mat histogramCols = cv::Mat::zeros(1, 16, CV_64FC1);
  unsigned long int counter = 0;

  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::Matx44d worldToCamera = niftk::RodriguesToMatrix(m_Rvecs[i], m_Tvecs[i]);
    m_Pipeline->SetWorldToCameraMatrix(worldToCamera);
    m_Pipeline->SetIntrinsics(m_Intrinsics);
    m_Pipeline->CopyScreen(m_RenderedImage);
    cv::cvtColor(m_RenderedImage, m_RenderedImageInGreyscale, CV_BGR2GRAY);

    this->AccumulateSamples(m_UndistortedVideoImagesInGreyScale[i], parameters[0],
                            counter, histogramRows, histogramCols, jointHist);
  }

  cost = this->ComputeNMI(counter, histogramRows, histogramCols, jointHist);

  return cost;
}


//-----------------------------------------------------------------------------
RenderingBasedMonoBlurringCostFunction::ParametersType
RenderingBasedMonoBlurringCostFunction::GetStepSizes() const
{
  ParametersType stepSize;
  stepSize.SetSize(this->GetNumberOfParameters());

  stepSize[0] = 1.0;    // sigma
  return stepSize;
}

} // end niftk
