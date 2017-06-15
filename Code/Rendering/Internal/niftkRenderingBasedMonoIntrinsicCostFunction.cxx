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
: m_Pipeline(nullptr)
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

  if (win == nullptr)
  {
    niftkNiftyCalThrow() << "Null Window provided.";
  }

  m_Pipeline.reset(new CalibratedRenderingPipeline(windowSize,
                                                   calibratedWindowSize,
                                                   model,
                                                   texture,
                                                   true));

  m_Pipeline->ConnectToRenderWindow(win);

  m_Rvecs = rvecs;
  m_Tvecs = tvecs;

  m_OriginalVideoImages = videoImages;
  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::Mat videoInGreyScale = cvCreateMat(m_OriginalVideoImages[i].rows, m_OriginalVideoImages[i].cols, CV_8UC3);
    cv::cvtColor(m_OriginalVideoImages[i], videoInGreyScale, CV_BGR2GRAY);
    m_OriginalVideoImagesInGreyScale.push_back(videoInGreyScale);

    cv::Mat undistorted;
    m_OriginalVideoImagesInGreyScale[i].copyTo(undistorted);
    m_UndistortedVideoImagesInGreyScale.push_back(undistorted);

    cv::Mat rendering;
    m_OriginalVideoImages[i].copyTo(rendering);
    m_RenderedImages.push_back(rendering);

    cv::Mat renderingInGreyScale;
    cv::cvtColor(m_RenderedImages[i], renderingInGreyScale, CV_BGR2GRAY);
    m_RenderedImagesInGreyscale.push_back(renderingInGreyScale);
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
  MeasureType cost = 0;
  cv::Mat jointHist = cv::Mat::zeros(32, 32, CV_64FC1);
  cv::Mat histogramRows = cv::Mat::zeros(32, 1, CV_64FC1);
  cv::Mat histogramCols = cv::Mat::zeros(1, 32, CV_64FC1);
  unsigned long int counter = 0;

  cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_64FC1);
  intrinsics.at<double>(0, 0) = parameters[0];
  intrinsics.at<double>(1, 1) = parameters[1];
  intrinsics.at<double>(0, 2) = parameters[2];
  intrinsics.at<double>(1, 2) = parameters[3];

  cv::Mat distortion = cv::Mat::zeros(1, 4, CV_64FC1);
  distortion.at<double>(0, 0) = parameters[4];
  distortion.at<double>(0, 1) = parameters[5];
  distortion.at<double>(0, 2) = parameters[6];
  distortion.at<double>(0, 3) = parameters[7];

  cv::Vec3i blueBackGround; // BGR
  blueBackGround[0] = 255;
  blueBackGround[1] = 0;
  blueBackGround[2] = 0;

  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::Matx44d worldToCamera = niftk::RodriguesToMatrix(m_Rvecs[i], m_Tvecs[i]);
    m_Pipeline->SetWorldToCameraMatrix(worldToCamera);
    m_Pipeline->SetIntrinsics(intrinsics);
    m_Pipeline->CopyScreen(m_RenderedImages[i]);

    cv::undistort(m_OriginalVideoImagesInGreyScale[i],
                  m_UndistortedVideoImagesInGreyScale[i],
                  intrinsics, distortion, intrinsics);

    for (int r = 0; r < m_OriginalVideoImages[i].rows; r++)
    {
      for (int c = 0; c < m_OriginalVideoImages[i].cols; c++)
      {
        if (   r != 0
            && c != 0
            && r != (m_RenderedImagesInGreyscale[i].rows - 1)
            && c != (m_RenderedImagesInGreyscale[i].cols - 1)
            && m_RenderedImages[i].at<cv::Vec3i>(r, c) != blueBackGround)
        {
          unsigned int a = static_cast<unsigned int>(m_RenderedImagesInGreyscale[i].at<unsigned char>(r, c)) / 8;
          unsigned int b = static_cast<unsigned int>(m_UndistortedVideoImagesInGreyScale[i].at<unsigned char>(r, c)) / 8;

          jointHist.at<double>(a, b) += 1;
          histogramRows.at<double>(a, 0) += 1;
          histogramCols.at<double>(0, b) += 1;
          counter += 1;
        }
      }
    }
  }

  // Now compute NMI
  double value = 0;
  double p = 0;
  double entropyRows = 0;
  double entropyCols = 0;
  double jointEntropy = 0;

  if (counter > 0)
  {
    for (int r = 0; r < histogramRows.rows; r++)
    {
      value = histogramRows.at<double>(r, 0);
      p = value / static_cast<double>(counter);
      if (p > 0)
      {
        entropyRows += p * log(p);
      }
    }

    for (int c = 0; c < histogramCols.cols; c++)
    {
      value = histogramCols.at<double>(0, c);
      p = value / static_cast<double>(counter);
      if (p > 0)
      {
        entropyCols += p * log(p);
      }
    }

    for (int r = 0; r < jointHist.rows; r++)
    {
      for (int c = 0; c < jointHist.cols; c++)
      {
        value = jointHist.at<double>(r, c);
        p = value / static_cast<double>(counter);
        if (p > 0)
        {
          jointEntropy += p * log(p);
        }
      }
    }
  }

  cost = (-entropyRows + -entropyCols)/-jointEntropy;

  return cost;
}


//-----------------------------------------------------------------------------
void RenderingBasedMonoIntrinsicCostFunction::GetDerivative(const ParametersType & parameters,
                                                            DerivativeType & derivative) const
{
  ParametersType stepSize;
  stepSize.SetSize(parameters.GetSize());
  stepSize[0] = 50;
  stepSize[1] = 50;
  stepSize[2] = 2;
  stepSize[3] = 2;
  stepSize[4] = 0.01;
  stepSize[5] = 0.01;
  stepSize[6] = 0.01;
  stepSize[7] = 0.01;

  derivative.SetSize(parameters.GetSize());

  for (int i = 0; i < parameters.GetSize(); i++)
  {
    ParametersType copyOfParams = parameters;

    copyOfParams[i] += stepSize[i];
    MeasureType plus = this->GetValue(copyOfParams);

    copyOfParams[i] -= (2.0 * stepSize[i]);
    MeasureType minus = this->GetValue(copyOfParams);

    MeasureType diff = plus - minus;
    derivative[i] = diff / (2.0*stepSize[i]);
  }
}

} // end niftk
