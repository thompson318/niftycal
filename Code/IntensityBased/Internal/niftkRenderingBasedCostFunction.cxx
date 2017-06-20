/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkRenderingBasedCostFunction.h"
#include <niftkMatrixUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>

namespace niftk
{

//-----------------------------------------------------------------------------
RenderingBasedCostFunction::RenderingBasedCostFunction()
: m_Pipeline(nullptr)
{
  m_BackgroundColour[0] = 255; // B
  m_BackgroundColour[1] = 0;   // G
  m_BackgroundColour[2] = 0;   // R
}


//-----------------------------------------------------------------------------
RenderingBasedCostFunction::~RenderingBasedCostFunction()
{
}


//-----------------------------------------------------------------------------
void RenderingBasedCostFunction::Initialise(vtkRenderWindow* win,
                                            const cv::Size2i& windowSize,
                                            const cv::Size2i& calibratedWindowSize,
                                            const std::string& model,
                                            const std::string& texture,
                                            const std::vector<cv::Mat>& videoImages
                                           )
{
  IntensityBasedCostFunction::Initialise(videoImages);

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
  m_Pipeline->SetBackgroundColour(m_BackgroundColour[2], m_BackgroundColour[1], m_BackgroundColour[0]); // OpenCV is BGR, VTK is RGB.

  m_OriginalVideoImages[0].copyTo(m_RenderedImage);
  cv::cvtColor(m_RenderedImage, m_RenderedImageInGreyscale, CV_BGR2GRAY);
}


//-----------------------------------------------------------------------------
void RenderingBasedCostFunction::AccumulateSamples(const cv::Mat& greyScaleVideoImage,
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
          && m_RenderedImage.at<cv::Vec3b>(r, c) != m_BackgroundColour)
      {
        unsigned int a = static_cast<unsigned int>(m_RenderedImageInGreyscale.at<unsigned char>(r, c)) / 16;
        unsigned int b = static_cast<unsigned int>(greyScaleVideoImage.at<unsigned char>(r, c)) / 16;

        jointHistogram.at<double>(a, b) += 1;
        histogramRows.at<double>(a, 0) += 1;
        histogramCols.at<double>(0, b) += 1;
        counter += 1;

      } // if not background
    } // end for cols
  } // end for rows
}

} // end niftk
