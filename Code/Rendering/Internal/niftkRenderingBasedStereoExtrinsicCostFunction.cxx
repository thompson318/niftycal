/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingBasedStereoExtrinsicCostFunction.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>

namespace niftk
{

//-----------------------------------------------------------------------------
RenderingBasedStereoExtrinsicCostFunction::RenderingBasedStereoExtrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
RenderingBasedStereoExtrinsicCostFunction::~RenderingBasedStereoExtrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
void RenderingBasedStereoExtrinsicCostFunction::Initialise(vtkRenderWindow* win,
                                                           const cv::Size2i& windowSize,
                                                           const cv::Size2i& calibratedWindowSize,
                                                           const std::string& model,
                                                           const std::string& texture,
                                                           const std::vector<cv::Mat>& leftVideoImages,
                                                           const std::vector<cv::Mat>& rightVideoImages,
                                                           const cv::Mat& leftIntrinsics,
                                                           const cv::Mat& leftDistortion,
                                                           const cv::Mat& rightIntrinsics,
                                                           const cv::Mat& rightDistortion
                                                          )
{
  Superclass::Initialise(win, windowSize, calibratedWindowSize, model, texture, leftVideoImages);

  if (leftVideoImages.size() != rightVideoImages.size())
  {
    niftkNiftyCalThrow() << "Number of left images (" << leftVideoImages.size()
                         << "), doesn't match number of right images (" << rightVideoImages.size() << ")";
  }

  m_LeftIntrinsics = leftIntrinsics;
  m_LeftDistortion = leftDistortion;
  m_RightIntrinsics = rightIntrinsics;
  m_RightDistortion = rightDistortion;

  m_RightOriginalVideoImages = rightVideoImages;
  for (int i = 0; i < m_RightOriginalVideoImages.size(); i++)
  {
    cv::Mat videoInGreyScale;
    cv::cvtColor(m_RightOriginalVideoImages[i], videoInGreyScale, CV_BGR2GRAY);
    m_RightOriginalVideoImagesInGreyScale.push_back(videoInGreyScale);

    cv::Mat undistorted;
    m_RightOriginalVideoImagesInGreyScale[i].copyTo(undistorted);
    m_RightUndistortedVideoImagesInGreyScale.push_back(undistorted);
  }

  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::undistort(m_OriginalVideoImagesInGreyScale[i],
                  m_UndistortedVideoImagesInGreyScale[i],
                  m_LeftIntrinsics, m_LeftDistortion, m_LeftIntrinsics);

    cv::undistort(m_RightOriginalVideoImagesInGreyScale[i],
                  m_RightUndistortedVideoImagesInGreyScale[i],
                  m_RightIntrinsics, m_RightDistortion, m_RightIntrinsics);
  }
}


//-----------------------------------------------------------------------------
unsigned int RenderingBasedStereoExtrinsicCostFunction::GetNumberOfParameters(void) const
{
  return 6 +
         m_OriginalVideoImages.size() * 6;
}


//-----------------------------------------------------------------------------
RenderingBasedStereoExtrinsicCostFunction::MeasureType
RenderingBasedStereoExtrinsicCostFunction::GetValue(const ParametersType & parameters) const
{  
  MeasureType cost = 0;
  cv::Mat jointHist = cv::Mat::zeros(32, 32, CV_64FC1);
  cv::Mat histogramRows = cv::Mat::zeros(32, 1, CV_64FC1);
  cv::Mat histogramCols = cv::Mat::zeros(1, 32, CV_64FC1);
  unsigned long int counter = 0;

  cv::Mat rvec = cv::Mat::zeros(1, 3, CV_64FC1);
  rvec.at<double>(0, 0) = parameters[0];
  rvec.at<double>(0, 1) = parameters[1];
  rvec.at<double>(0, 2) = parameters[2];

  cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64FC1);
  tvec.at<double>(0, 0) = parameters[3];
  tvec.at<double>(0, 1) = parameters[4];
  tvec.at<double>(0, 2) = parameters[5];

  cv::Matx44d leftToRight = niftk::RotationAndTranslationToMatrix(rvec, tvec);

  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::Mat rotation = cv::Mat::zeros(1, 3, CV_64FC1);
    rotation.at<double>(0, 0) = parameters[(i+1)*6 + 0];
    rotation.at<double>(0, 1) = parameters[(i+1)*6 + 1];
    rotation.at<double>(0, 2) = parameters[(i+1)*6 + 2];

    cv::Mat translation = cv::Mat::zeros(1, 3, CV_64FC1);
    translation.at<double>(0, 0) = parameters[(i+1)*6 + 3];
    translation.at<double>(0, 1) = parameters[(i+1)*6 + 4];
    translation.at<double>(0, 2) = parameters[(i+1)*6 + 5];

    // Render left
    cv::Matx44d worldToLeftCamera = niftk::RodriguesToMatrix(rotation, translation);
    m_Pipeline->SetWorldToCameraMatrix(worldToLeftCamera);
    m_Pipeline->SetIntrinsics(m_LeftIntrinsics);
    m_Pipeline->CopyScreen(m_RenderedImage);
    cv::cvtColor(m_RenderedImage, m_RenderedImageInGreyscale, CV_BGR2GRAY);
    this->AccumulateSamples(m_UndistortedVideoImagesInGreyScale[i],
                            counter, histogramRows, histogramCols, jointHist);

    // Render right
    cv::Matx44d worldToRightCamera = leftToRight * worldToLeftCamera;
    m_Pipeline->SetWorldToCameraMatrix(worldToRightCamera);
    m_Pipeline->SetIntrinsics(m_RightIntrinsics);
    m_Pipeline->CopyScreen(m_RenderedImage);
    cv::cvtColor(m_RenderedImage, m_RenderedImageInGreyscale, CV_BGR2GRAY);
    this->AccumulateSamples(m_RightUndistortedVideoImagesInGreyScale[i],
                            counter, histogramRows, histogramCols, jointHist);

  }

  cost = this->ComputeNMI(counter, histogramRows, histogramCols, jointHist);

  return cost;
}


//-----------------------------------------------------------------------------
RenderingBasedStereoExtrinsicCostFunction::ParametersType
RenderingBasedStereoExtrinsicCostFunction::GetStepSizes() const
{
  ParametersType stepSize;
  stepSize.SetSize(this->GetNumberOfParameters());

  for (int i = 0; i < m_OriginalVideoImages.size()+1; i++)
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
