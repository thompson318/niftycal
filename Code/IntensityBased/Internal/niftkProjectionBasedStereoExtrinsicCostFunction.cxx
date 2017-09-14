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
  Superclass::Initialise(windowSize, model, 16);

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

  cv::Rodrigues(leftToRightRotationMatrix, m_LeftToRightRVec);
  m_LeftToRightTVec = leftToRightTranslationVector;

  m_LeftImages = leftVideoImages;
  m_RightImages = rightVideoImages;

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
  return 2 + // JUST do x and y translation.
        (6 * m_LeftImagesInGreyScale.size());
}


//-----------------------------------------------------------------------------
ProjectionBasedStereoExtrinsicCostFunction::MeasureType
ProjectionBasedStereoExtrinsicCostFunction::GetValue(const ParametersType & parameters) const
{
  MeasureType cost = 0;
  cv::Mat leftJointHist = cv::Mat::zeros(16, 16, CV_64FC1);
  cv::Mat leftHistogramRows = cv::Mat::zeros(16, 1, CV_64FC1);
  cv::Mat leftHistogramCols = cv::Mat::zeros(1, 16, CV_64FC1);
  unsigned long int leftCounter = 0;

  cv::Mat rightJointHist = cv::Mat::zeros(16, 16, CV_64FC1);
  cv::Mat rightHistogramRows = cv::Mat::zeros(16, 1, CV_64FC1);
  cv::Mat rightHistogramCols = cv::Mat::zeros(1, 16, CV_64FC1);
  unsigned long int rightCounter = 0;

  cv::Mat rvec = cv::Mat::zeros(1, 3, CV_64FC1);
  rvec.at<double>(0, 0) = m_LeftToRightRVec.at<double>(0, 0);
  rvec.at<double>(0, 1) = m_LeftToRightRVec.at<double>(0, 1);
  rvec.at<double>(0, 2) = m_LeftToRightRVec.at<double>(0, 2);

  cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64FC1);
  tvec.at<double>(0, 0) = parameters[0];                       // i.e. only optimise t_x, t_y.
  tvec.at<double>(0, 1) = parameters[1];
  tvec.at<double>(0, 2) = m_LeftToRightTVec.at<double>(2, 0);

  cv::Matx44d leftToRight = niftk::RodriguesToMatrix(rvec, tvec);

  for (int leftA = 0; leftA < m_LeftImagesInGreyScale.size(); leftA++)
  {
    cv::Mat rvecLeftA = cv::Mat::zeros(1, 3, CV_64FC1);
    rvecLeftA.at<double>(0, 0) = parameters[(6*leftA)+2];
    rvecLeftA.at<double>(0, 1) = parameters[(6*leftA)+3];
    rvecLeftA.at<double>(0, 2) = parameters[(6*leftA)+4];

    cv::Mat tvecLeftA = cv::Mat::zeros(1, 3, CV_64FC1);
    tvecLeftA.at<double>(0, 0) = parameters[(6*leftA)+5];
    tvecLeftA.at<double>(0, 1) = parameters[(6*leftA)+6];
    tvecLeftA.at<double>(0, 2) = parameters[(6*leftA)+7];

    for (int leftB = 0; leftB < m_LeftImagesInGreyScale.size(); leftB++)
    {
      cv::Mat rvecLeftB = cv::Mat::zeros(1, 3, CV_64FC1);
      rvecLeftB.at<double>(0, 0) = parameters[(6*leftB)+2];
      rvecLeftB.at<double>(0, 1) = parameters[(6*leftB)+3];
      rvecLeftB.at<double>(0, 2) = parameters[(6*leftB)+4];

      cv::Mat tvecLeftB = cv::Mat::zeros(1, 3, CV_64FC1);
      tvecLeftB.at<double>(0, 0) = parameters[(6*leftB)+5];
      tvecLeftB.at<double>(0, 1) = parameters[(6*leftB)+6];
      tvecLeftB.at<double>(0, 2) = parameters[(6*leftB)+7];

      if (leftA != leftB)
      {
        this->AccumulateSamples(m_LeftImagesInGreyScale[leftA],
                                m_LeftIntrinsics,
                                m_LeftDistortion,
                                rvecLeftA,
                                tvecLeftA,
                                m_LeftImagesInGreyScale[leftB],
                                m_LeftIntrinsics,
                                m_LeftDistortion,
                                rvecLeftB,
                                tvecLeftB,
                                leftCounter,
                                leftHistogramRows,
                                leftHistogramCols,
                                leftJointHist
                                );

        cv::Matx44d leftCameraA = niftk::RodriguesToMatrix(rvecLeftA, tvecLeftA);
        cv::Matx44d rightCameraA = leftToRight * leftCameraA;

        cv::Mat rvecRightA = cv::Mat::zeros(1, 3, CV_64FC1);
        cv::Mat tvecRightA = cv::Mat::zeros(1, 3, CV_64FC1);
        niftk::MatrixToRodrigues(rightCameraA, rvecRightA, tvecRightA);

        cv::Matx44d leftCameraB = niftk::RodriguesToMatrix(rvecLeftB, tvecLeftB);
        cv::Matx44d rightCameraB = leftToRight * leftCameraB;

        cv::Mat rvecRightB = cv::Mat::zeros(1, 3, CV_64FC1);
        cv::Mat tvecRightB = cv::Mat::zeros(1, 3, CV_64FC1);
        niftk::MatrixToRodrigues(rightCameraB, rvecRightB, tvecRightB);

        this->AccumulateSamples(m_RightImagesInGreyScale[leftA],
                                m_RightIntrinsics,
                                m_RightDistortion,
                                rvecRightA,
                                tvecRightA,
                                m_RightImagesInGreyScale[leftB],
                                m_RightIntrinsics,
                                m_RightDistortion,
                                rvecRightB,
                                tvecRightB,
                                rightCounter,
                                rightHistogramRows,
                                rightHistogramCols,
                                rightJointHist
                                );

      } // end if leftA != leftB
    }
  }

  double costLeft = this->ComputeNMI(leftCounter, leftHistogramRows, leftHistogramCols, leftJointHist);
  double costRight = this->ComputeNMI(rightCounter, rightHistogramRows, rightHistogramCols, rightJointHist);

  return costLeft + costRight;
}


//-----------------------------------------------------------------------------
ProjectionBasedStereoExtrinsicCostFunction::ParametersType
ProjectionBasedStereoExtrinsicCostFunction::GetStepSizes() const
{
  ParametersType stepSize;
  stepSize.SetSize(this->GetNumberOfParameters());

  stepSize[0] = 0.1; // x-translation.
  stepSize[1] = 0.1; // y-translation.

  for (int i = 0; i < m_LeftImagesInGreyScale.size(); i++)
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
