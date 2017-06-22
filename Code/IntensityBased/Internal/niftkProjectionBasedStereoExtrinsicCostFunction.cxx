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
                                                            const cv::Mat& leftToRightRotationMatrix,
                                                            const cv::Mat& leftToRightTranslationVector
                                                           )
{
  Superclass::Initialise(windowSize, model);

  if (leftVideoImages.size() != rightVideoImages.size())
  {
    niftkNiftyCalThrow() << "Number of left images (" << leftVideoImages.size()
                         << "), doesn't match number of right images (" << rightVideoImages.size() << ")";
  }

  m_LeftIntrinsics = leftIntrinsics;
  m_LeftDistortion = leftDistortion;
  m_RightIntrinsics = rightIntrinsics;
  m_RightDistortion = rightDistortion;

  cv::Rodrigues(leftToRightRotationMatrix, m_LeftToRightRVec);
  m_LeftToRightTVec = leftToRightTranslationVector;
}


//-----------------------------------------------------------------------------
unsigned int ProjectionBasedStereoExtrinsicCostFunction::GetNumberOfParameters(void) const
{
  return 2 + // JUST do x and y translation.
         (6 * m_LeftImages.size());
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
  rvec.at<double>(0, 0) = m_LeftToRightRVec.at<double>(0, 0);
  rvec.at<double>(0, 1) = m_LeftToRightRVec.at<double>(0, 1);
  rvec.at<double>(0, 2) = m_LeftToRightRVec.at<double>(0, 2);

  cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64FC1);
  tvec.at<double>(0, 0) = parameters[0];
  tvec.at<double>(0, 1) = parameters[1];
  tvec.at<double>(0, 2) = m_LeftToRightTVec.at<double>(0, 2);

  cv::Matx44d leftToRight = niftk::RodriguesToMatrix(rvec, tvec);

  for (int i = 0; i < m_LeftImages.size(); i++)
  {
    cv::Mat rvecLeft = cv::Mat::zeros(1, 3, CV_64FC1);
    rvecLeft.at<double>(0, 0) = parameters[(6*i)+2];
    rvecLeft.at<double>(0, 1) = parameters[(6*i)+3];
    rvecLeft.at<double>(0, 2) = parameters[(6*i)+4];

    cv::Mat tvecLeft = cv::Mat::zeros(1, 3, CV_64FC1);
    tvecLeft.at<double>(0, 0) = parameters[(6*i)+5];
    tvecLeft.at<double>(0, 1) = parameters[(6*i)+6];
    tvecLeft.at<double>(0, 2) = parameters[(6*i)+7];

    cv::Matx44d leftCamera = niftk::RodriguesToMatrix(rvecLeft, tvecLeft);
    cv::Matx44d rightCamera = leftToRight * leftCamera;

    cv::Mat rvecRight = cv::Mat::zeros(1, 3, CV_64FC1);
    cv::Mat tvecRight = cv::Mat::zeros(1, 3, CV_64FC1);
    niftk::MatrixToRodrigues(rightCamera, rvecRight, tvecRight);

    this->AccumulateSamples(m_LeftImages[i],
                            m_LeftIntrinsics,
                            m_LeftDistortion,
                            rvecLeft,
                            tvecLeft,
                            m_RightImages[i],
                            m_RightIntrinsics,
                            m_RightDistortion,
                            rvecRight,
                            tvecRight,
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

  stepSize[0] = 0.1; // x-translation.
  stepSize[1] = 0.1; // y-translation.

  for (int i = 0; i < m_LeftImages.size(); i++)
  {
    stepSize[6*i + 2] = 0.01; // r1 (Rodrigues)
    stepSize[6*i + 3] = 0.01; // r2 (Rodrigues)
    stepSize[6*i + 4] = 0.01; // r3 (Rodrigues)
    stepSize[6*i + 5] = 0.1;  // tx
    stepSize[6*i + 6] = 0.1;  // ty
    stepSize[6*i + 7] = 0.1;  // tz
  }

  return stepSize;
}

} // end niftk
