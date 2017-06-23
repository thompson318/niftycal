/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkProjectionBasedStereoExtrinsicCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <niftkMatrixUtilities.h>

namespace niftk
{

//-----------------------------------------------------------------------------
ProjectionBasedStereoExtrinsicCostFunction::ProjectionBasedStereoExtrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
ProjectionBasedStereoExtrinsicCostFunction::~ProjectionBasedStereoExtrinsicCostFunction()
{
}


//-----------------------------------------------------------------------------
void ProjectionBasedStereoExtrinsicCostFunction::Initialise(const cv::Size2i& windowSize,
                                                            const std::string& model,
                                                            const std::vector<cv::Mat>& leftVideoImages,
                                                            const std::vector<cv::Mat>& rightVideoImages,
                                                            const cv::Mat& leftIntrinsics,
                                                            const cv::Mat& leftDistortion,
                                                            const cv::Mat& rightIntrinsics,
                                                            const cv::Mat& rightDistortion,
                                                            const std::vector<cv::Mat>& rvecLeft,
                                                            const std::vector<cv::Mat>& tvecLeft
                                                           )
{
  Superclass::Initialise(windowSize, model);

  if (leftVideoImages.size() != rightVideoImages.size())
  {
    niftkNiftyCalThrow() << "Number of left images (" << leftVideoImages.size()
                         << "), doesn't match number of right images (" << rightVideoImages.size() << ")";
  }

  if (leftVideoImages.size() <= 1)
  {
    niftkNiftyCalThrow() << "Too few left video images.";
  }

  if (rightVideoImages.size() <= 1)
  {
    niftkNiftyCalThrow() << "Too few right video images.";
  }

  m_LeftIntrinsics = leftIntrinsics;
  m_LeftDistortion = leftDistortion;

  m_RightIntrinsics = rightIntrinsics;
  m_RightDistortion = rightDistortion;

  m_LeftImages = leftVideoImages;
  m_RightImages = rightVideoImages;

  m_RVecLeft = rvecLeft;
  m_TVecLeft = tvecLeft;

  for (int i = 0; i < leftVideoImages.size(); i++)
  {
    cv::Mat greyImage;
    cv::cvtColor(m_LeftImages[i], greyImage, CV_BGR2GRAY);
    m_LeftImagesInGreyScale.push_back(greyImage);
  }

  for (int i = 0; i < rightVideoImages.size(); i++)
  {
    cv::Mat greyImage;
    cv::cvtColor(m_RightImages[i], greyImage, CV_BGR2GRAY);
    m_RightImagesInGreyScale.push_back(greyImage);
  }
}


//-----------------------------------------------------------------------------
unsigned int ProjectionBasedStereoExtrinsicCostFunction::GetNumberOfParameters(void) const
{
  return 6;
}


//-----------------------------------------------------------------------------
ProjectionBasedStereoExtrinsicCostFunction::MeasureType
ProjectionBasedStereoExtrinsicCostFunction::GetValue(const ParametersType & parameters) const
{
  MeasureType cost = 0;

  cv::Mat jointHist = cv::Mat::zeros(16, 16, CV_64FC1);
  cv::Mat histogramRows = cv::Mat::zeros(16, 1, CV_64FC1);
  cv::Mat histogramCols = cv::Mat::zeros(1, 16, CV_64FC1);
  unsigned long int counter = 0;

  cv::Mat rvec = cv::Mat::zeros(1, 3, CV_64FC1);
  rvec.at<double>(0, 0) = parameters[0];
  rvec.at<double>(0, 1) = parameters[1];
  rvec.at<double>(0, 2) = parameters[2];

  cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64FC1);
  tvec.at<double>(0, 0) = parameters[3];
  tvec.at<double>(0, 1) = parameters[4];
  tvec.at<double>(0, 2) = parameters[5];

  cv::Matx44d leftToRight = niftk::RodriguesToMatrix(rvec, tvec);

  for (int i = 0; i < m_RightImages.size() - 1; i++)
  {
    cv::Matx44d leftCameraA = niftk::RodriguesToMatrix(m_RVecLeft[i], m_TVecLeft[i]);
    cv::Matx44d leftCameraB = niftk::RodriguesToMatrix(m_RVecLeft[i+1], m_TVecLeft[i+1]);

    cv::Matx44d rightCameraA = leftToRight * leftCameraA;
    cv::Matx44d rightCameraB = leftToRight * leftCameraB;

    cv::Mat rvecRightA = cv::Mat::zeros(1, 3, CV_64FC1);
    cv::Mat tvecRightA = cv::Mat::zeros(1, 3, CV_64FC1);
    niftk::MatrixToRodrigues(rightCameraA, rvecRightA, tvecRightA);

    cv::Mat rvecRightB = cv::Mat::zeros(1, 3, CV_64FC1);
    cv::Mat tvecRightB = cv::Mat::zeros(1, 3, CV_64FC1);
    niftk::MatrixToRodrigues(rightCameraB, rvecRightB, tvecRightB);

    this->AccumulateSamples(m_RightImagesInGreyScale[i],
                            m_RightIntrinsics,
                            m_RightDistortion,
                            rvecRightA,
                            tvecRightA,
                            m_RightImagesInGreyScale[i+1],
                            m_RightIntrinsics,
                            m_RightDistortion,
                            rvecRightB,
                            tvecRightB,
                            counter,
                            histogramRows,
                            histogramCols,
                            jointHist
                            );
  }

  cost = this->ComputeNMI(counter, histogramRows, histogramCols, jointHist);

  return cost;
}


//-----------------------------------------------------------------------------
ProjectionBasedStereoExtrinsicCostFunction::ParametersType
ProjectionBasedStereoExtrinsicCostFunction::GetStepSizes() const
{
  ParametersType stepSize;
  stepSize.SetSize(this->GetNumberOfParameters());

  stepSize[0] = 0.01; // r1 (Rodrigues)
  stepSize[1] = 0.01; // r2 (Rodrigues)
  stepSize[2] = 0.01; // r3 (Rodrigues)
  stepSize[3] = 0.1;  // tx
  stepSize[4] = 0.1;  // ty
  stepSize[5] = 0.1;  // tz

  return stepSize;
}

} // end niftk
