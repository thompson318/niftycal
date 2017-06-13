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
    cv::Mat videoInGreyScale;
    cv::cvtColor(m_OriginalVideoImages[i], videoInGreyScale, CV_BGR2GRAY);
    m_OriginalVideoImagesInGreyScale.push_back(videoInGreyScale);

    cv::Mat undistorted;
    m_OriginalVideoImagesInGreyScale[i].copyTo(undistorted);
    m_UndistortedVideoImagesInGreyScale.push_back(undistorted);

    cv::Mat rendering;
    m_OriginalVideoImages[i].copyTo(rendering);
    m_RenderedImages.push_back(rendering);

    cv::Matx44d worldToCamera = niftk::RodriguesToMatrix(rvecs[i], tvecs[i]);
    m_Pipeline->SetWorldToCameraMatrix(worldToCamera);

    m_Pipeline->DumpScreen(m_RenderedImages[i]);

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

  for (int i = 0; i < m_OriginalVideoImages.size(); i++)
  {
    cv::undistort(m_OriginalVideoImagesInGreyScale[i],
                  m_UndistortedVideoImagesInGreyScale[i],
                  intrinsics, distortion, intrinsics);
  }

  return cost;
}


//-----------------------------------------------------------------------------
void RenderingBasedMonoIntrinsicCostFunction::GetDerivative(const ParametersType & parameters,
                                                            DerivativeType & derivative) const
{
}

} // end niftk
