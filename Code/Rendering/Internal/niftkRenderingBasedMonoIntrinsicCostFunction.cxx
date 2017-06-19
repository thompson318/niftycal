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
#include <highgui.h>

namespace niftk
{

//-----------------------------------------------------------------------------
RenderingBasedMonoIntrinsicCostFunction::RenderingBasedMonoIntrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
RenderingBasedMonoIntrinsicCostFunction::~RenderingBasedMonoIntrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
void RenderingBasedMonoIntrinsicCostFunction::Initialise(vtkRenderWindow* win,
                                                         const cv::Size2i& windowSize,
                                                         const cv::Size2i& calibratedWindowSize,
                                                         const std::string& model,
                                                         const std::string& texture,
                                                         const std::vector<cv::Mat>& videoImages,
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

  m_Rvecs = rvecs;
  m_Tvecs = tvecs;
}


//-----------------------------------------------------------------------------
unsigned int RenderingBasedMonoIntrinsicCostFunction::GetNumberOfParameters(void) const
{
  return 9;
}


//-----------------------------------------------------------------------------
RenderingBasedMonoIntrinsicCostFunction::MeasureType
RenderingBasedMonoIntrinsicCostFunction::GetValue(const ParametersType & parameters) const
{  
  MeasureType cost = 0;
  cv::Mat jointHist = cv::Mat::zeros(16, 16, CV_64FC1);
  cv::Mat histogramRows = cv::Mat::zeros(16, 1, CV_64FC1);
  cv::Mat histogramCols = cv::Mat::zeros(1, 16, CV_64FC1);
  unsigned long int counter = 0;

  cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_64FC1);
  intrinsics.at<double>(0, 0) = parameters[0];
  intrinsics.at<double>(1, 1) = parameters[1];
  intrinsics.at<double>(0, 2) = parameters[2];
  intrinsics.at<double>(1, 2) = parameters[3];

  cv::Mat distortion = cv::Mat::zeros(1, 5, CV_64FC1);
  distortion.at<double>(0, 0) = parameters[4];
  distortion.at<double>(0, 1) = parameters[5];
  distortion.at<double>(0, 2) = parameters[6];
  distortion.at<double>(0, 3) = parameters[7];
  distortion.at<double>(0, 4) = parameters[8];

  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::Matx44d worldToCamera = niftk::RodriguesToMatrix(m_Rvecs[i], m_Tvecs[i]);
    m_Pipeline->SetWorldToCameraMatrix(worldToCamera);
    m_Pipeline->SetIntrinsics(intrinsics);
    m_Pipeline->CopyScreen(m_RenderedImage);
    cv::cvtColor(m_RenderedImage, m_RenderedImageInGreyscale, CV_BGR2GRAY);

    cv::undistort(m_OriginalVideoImagesInGreyScale[i],
                  m_UndistortedVideoImagesInGreyScale[i],
                  intrinsics, distortion, intrinsics);

    this->AccumulateSamples(m_UndistortedVideoImagesInGreyScale[i],
                            counter, histogramRows, histogramCols, jointHist);
  }

  cost = this->ComputeNMI(counter, histogramRows, histogramCols, jointHist);

  return cost;
}


//-----------------------------------------------------------------------------
RenderingBasedMonoIntrinsicCostFunction::ParametersType
RenderingBasedMonoIntrinsicCostFunction::GetStepSizes() const
{
  ParametersType stepSize;
  stepSize.SetSize(this->GetNumberOfParameters());

  stepSize[0] = 50;    // fx
  stepSize[1] = 50;    // fy
  stepSize[2] = 2;     // cx
  stepSize[3] = 2;     // cy
  stepSize[4] = 0.01;  // k1
  stepSize[5] = 0.01;  // etc.
  stepSize[6] = 0.01;
  stepSize[7] = 0.01;
  stepSize[8] = 0.01;

  return stepSize;
}

} // end niftk
