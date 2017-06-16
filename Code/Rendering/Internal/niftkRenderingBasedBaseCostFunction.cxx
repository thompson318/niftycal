/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingBasedBaseCostFunction.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>
#include <highgui.h>

namespace niftk
{

//-----------------------------------------------------------------------------
RenderingBasedBaseCostFunction::RenderingBasedBaseCostFunction()
: m_Pipeline(nullptr)
{
  m_BackgroundColour[0] = 255; // B
  m_BackgroundColour[1] = 0;   // G
  m_BackgroundColour[2] = 0;   // R
}


//-----------------------------------------------------------------------------
RenderingBasedBaseCostFunction::~RenderingBasedBaseCostFunction()
{
}


//-----------------------------------------------------------------------------
void RenderingBasedBaseCostFunction::Initialise(vtkRenderWindow* win,
                                                const cv::Size2i& windowSize,
                                                const cv::Size2i& calibratedWindowSize,
                                                const std::string& model,
                                                const std::string& texture,
                                                const std::vector<cv::Mat>& videoImages
                                               )
{
  if (win == nullptr)
  {
    niftkNiftyCalThrow() << "Null Window provided.";
  }

  if (videoImages.size() == 0)
  {
    niftkNiftyCalThrow() << "No images provided.";
  }

  m_Pipeline.reset(new CalibratedRenderingPipeline(windowSize,
                                                   calibratedWindowSize,
                                                   model,
                                                   texture,
                                                   true));

  m_Pipeline->ConnectToRenderWindow(win);
  m_Pipeline->SetBackgroundColour(m_BackgroundColour[2], m_BackgroundColour[1], m_BackgroundColour[0]); // OpenCV is BGR, VTK is RGB.

  m_OriginalVideoImages = videoImages;
  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::Mat videoInGreyScale;
    cv::cvtColor(m_OriginalVideoImages[i], videoInGreyScale, CV_BGR2GRAY);
    m_OriginalVideoImagesInGreyScale.push_back(videoInGreyScale);

    cv::Mat undistorted;
    m_OriginalVideoImagesInGreyScale[i].copyTo(undistorted);
    m_UndistortedVideoImagesInGreyScale.push_back(undistorted);
  }

  m_OriginalVideoImages[0].copyTo(m_RenderedImage);
  cv::cvtColor(m_RenderedImage, m_RenderedImageInGreyscale, CV_BGR2GRAY);

}


//-----------------------------------------------------------------------------
void RenderingBasedBaseCostFunction::AccumulateSamples(const cv::Mat& greyScaleVideoImage,
                                                       unsigned long int& counter,
                                                       cv::Mat& histogramRows,
                                                       cv::Mat& histogramCols,
                                                       cv::Mat& jointHistogram
                                                      ) const
{
  for (int r = 0; r < m_RenderedImageInGreyscale.rows; r++)
  {
    for (int c = 0; c < m_RenderedImageInGreyscale.cols; c++)
    {
      if (   r != 0
          && c != 0
          && r != (m_RenderedImageInGreyscale.rows - 1)
          && c != (m_RenderedImageInGreyscale.cols - 1)
          && m_RenderedImage.at<cv::Vec3i>(r, c) != m_BackgroundColour)
      {
        unsigned int a = static_cast<unsigned int>(m_RenderedImageInGreyscale.at<unsigned char>(r, c)) / 8;
        unsigned int b = static_cast<unsigned int>(greyScaleVideoImage.at<unsigned char>(r, c)) / 8;

        jointHistogram.at<double>(a, b) += 1;
        histogramRows.at<double>(a, 0) += 1;
        histogramCols.at<double>(0, b) += 1;
        counter += 1;

      } // if not background
    } // end for cols
  } // end for rows
}


//-----------------------------------------------------------------------------
double RenderingBasedBaseCostFunction::ComputeNMI(const unsigned long int& counter,
                                                  const cv::Mat& histogramRows,
                                                  const cv::Mat& histogramCols,
                                                  const cv::Mat& jointHist
                                                 ) const
{
  MeasureType cost = 0;
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

    cost = (-entropyRows + -entropyCols)/-jointEntropy;
  }

  return cost;
}


//-----------------------------------------------------------------------------
void RenderingBasedBaseCostFunction::GetDerivative(const ParametersType & parameters,
                                                   DerivativeType & derivative) const
{
  ParametersType stepSizes = this->GetStepSizes(); // from derived class
  derivative.SetSize(this->GetNumberOfParameters());

  if (parameters.GetSize() != stepSizes.GetSize())
  {
    niftkNiftyCalThrow() << "Unequal array sizes.";
  }
  if (parameters.GetSize() != derivative.GetSize())
  {
    niftkNiftyCalThrow() << "Unequal array sizes.";
  }

  for (int i = 0; i < parameters.GetSize(); i++)
  {
    ParametersType copyOfParams = parameters;

    copyOfParams[i] += stepSizes[i];
    MeasureType plus = this->GetValue(copyOfParams);

    copyOfParams[i] -= (2.0 * stepSizes[i]);
    MeasureType minus = this->GetValue(copyOfParams);

    MeasureType diff = plus - minus;
    derivative[i] = diff / (2.0*stepSizes[i]);
  }
}

} // end niftk
