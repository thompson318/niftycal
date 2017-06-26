/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkProjectionBasedMonoExtrinsicCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
ProjectionBasedMonoExtrinsicCostFunction::ProjectionBasedMonoExtrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
ProjectionBasedMonoExtrinsicCostFunction::~ProjectionBasedMonoExtrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
void ProjectionBasedMonoExtrinsicCostFunction::Initialise(const cv::Size2i& windowSize,
                                                          const std::string& model,
                                                          const std::vector<cv::Mat>& videoImages,
                                                          const cv::Mat& intrinsics,
                                                          const cv::Mat& distortion
                                                         )
{
  Superclass::Initialise(windowSize, model, 16);

  if (videoImages.size() <= 1)
  {
    niftkNiftyCalThrow() << "Too few left video images.";
  }

  m_Intrinsics = intrinsics;
  m_Distortion = distortion;

  m_Images = videoImages;

  for (int i = 0; i < videoImages.size(); i++)
  {
    cv::Mat greyImage;
    cv::cvtColor(m_Images[i], greyImage, CV_BGR2GRAY);
    m_ImagesInGreyScale.push_back(greyImage);
  }
}


//-----------------------------------------------------------------------------
unsigned int ProjectionBasedMonoExtrinsicCostFunction::GetNumberOfParameters(void) const
{
  return 6 * m_ImagesInGreyScale.size();
}


//-----------------------------------------------------------------------------
ProjectionBasedMonoExtrinsicCostFunction::MeasureType
ProjectionBasedMonoExtrinsicCostFunction::GetValue(const ParametersType & parameters) const
{
  MeasureType cost = 0;
  cv::Mat jointHist = cv::Mat::zeros(16, 16, CV_64FC1);
  cv::Mat histogramRows = cv::Mat::zeros(16, 1, CV_64FC1);
  cv::Mat histogramCols = cv::Mat::zeros(1, 16, CV_64FC1);
  unsigned long int counter = 0;

  for (int i = 0; i < m_ImagesInGreyScale.size(); i++)
  {
    for (int j = 0; j < m_ImagesInGreyScale.size(); j++)
    {
      // At registration, the grey value of a projected model point
      // should be related to all other grey values found by projecting the
      // same model point onto all other left hand views.
      if (i != j)
      {
        cv::Mat rvecI = cv::Mat::zeros(1, 3, CV_64FC1);
        rvecI.at<double>(0, 0) = parameters[(6*i)+0];
        rvecI.at<double>(0, 1) = parameters[(6*i)+1];
        rvecI.at<double>(0, 2) = parameters[(6*i)+2];

        cv::Mat tvecI = cv::Mat::zeros(1, 3, CV_64FC1);
        tvecI.at<double>(0, 0) = parameters[(6*i)+3];
        tvecI.at<double>(0, 1) = parameters[(6*i)+4];
        tvecI.at<double>(0, 2) = parameters[(6*i)+5];

        cv::Mat rvecJ = cv::Mat::zeros(1, 3, CV_64FC1);
        rvecJ.at<double>(0, 0) = parameters[(6*j)+0];
        rvecJ.at<double>(0, 1) = parameters[(6*j)+1];
        rvecJ.at<double>(0, 2) = parameters[(6*j)+2];

        cv::Mat tvecJ = cv::Mat::zeros(1, 3, CV_64FC1);
        tvecJ.at<double>(0, 0) = parameters[(6*j)+3];
        tvecJ.at<double>(0, 1) = parameters[(6*j)+4];
        tvecJ.at<double>(0, 2) = parameters[(6*j)+5];

        this->AccumulateSamples(m_ImagesInGreyScale[i],
                                m_Intrinsics,
                                m_Distortion,
                                rvecI,
                                tvecI,
                                m_ImagesInGreyScale[j],
                                m_Intrinsics,
                                m_Distortion,
                                rvecJ,
                                tvecJ,
                                counter,
                                histogramRows,
                                histogramCols,
                                jointHist
                                );
      }
    }
  }

  cost = this->ComputeNMI(counter, histogramRows, histogramCols, jointHist);

  return cost;
}


//-----------------------------------------------------------------------------
ProjectionBasedMonoExtrinsicCostFunction::ParametersType
ProjectionBasedMonoExtrinsicCostFunction::GetStepSizes() const
{
  ParametersType stepSize;
  stepSize.SetSize(this->GetNumberOfParameters());

  for (int i = 0; i < m_ImagesInGreyScale.size(); i++)
  {
    stepSize[6*i + 0] = 0.01; // r1 (Rodrigues)
    stepSize[6*i + 1] = 0.01; // r2 (Rodrigues)
    stepSize[6*i + 2] = 0.01; // r3 (Rodrigues)
    stepSize[6*i + 3] = 0.1;  // tx
    stepSize[6*i + 4] = 0.1;  // ty
    stepSize[6*i + 5] = 0.1;  // tz
  }

  return stepSize;
}

} // end niftk
