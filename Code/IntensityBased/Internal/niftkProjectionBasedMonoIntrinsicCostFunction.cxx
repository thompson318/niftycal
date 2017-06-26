/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkProjectionBasedMonoIntrinsicCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
ProjectionBasedMonoIntrinsicCostFunction::ProjectionBasedMonoIntrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
ProjectionBasedMonoIntrinsicCostFunction::~ProjectionBasedMonoIntrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
void ProjectionBasedMonoIntrinsicCostFunction::Initialise(const cv::Size2i& windowSize,
                                                          const std::string& model,
                                                          const std::vector<cv::Mat>& videoImages,
                                                          const std::vector<cv::Mat>& rvecs,
                                                          const std::vector<cv::Mat>& tvecs
                                                         )
{
  Superclass::Initialise(windowSize, model);

  if (videoImages.size() <= 1)
  {
    niftkNiftyCalThrow() << "Too few left video images.";
  }

  m_Rvecs = rvecs;
  m_Tvecs = tvecs;

  m_Images = videoImages;

  for (int i = 0; i < videoImages.size(); i++)
  {
    cv::Mat greyImage;
    cv::cvtColor(m_Images[i], greyImage, CV_BGR2GRAY);
    m_ImagesInGreyScale.push_back(greyImage);
  }
}


//-----------------------------------------------------------------------------
unsigned int ProjectionBasedMonoIntrinsicCostFunction::GetNumberOfParameters(void) const
{
  return 9;
}


//-----------------------------------------------------------------------------
ProjectionBasedMonoIntrinsicCostFunction::MeasureType
ProjectionBasedMonoIntrinsicCostFunction::GetValue(const ParametersType & parameters) const
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

  for (int i = 0; i < m_ImagesInGreyScale.size(); i++)
  {
    for (int j = 0; j < m_ImagesInGreyScale.size(); j++)
    {
      // At registration, the grey value of a projected model point
      // should be related to all other grey values found by projecting the
      // same model point onto all other left hand views.
      if (i != j)
      {
        this->AccumulateSamples(m_ImagesInGreyScale[i],
                                intrinsics,
                                distortion,
                                m_Rvecs[i],
                                m_Tvecs[i],
                                m_ImagesInGreyScale[j],
                                intrinsics,
                                distortion,
                                m_Rvecs[j],
                                m_Tvecs[j],
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
ProjectionBasedMonoIntrinsicCostFunction::ParametersType
ProjectionBasedMonoIntrinsicCostFunction::GetStepSizes() const
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
