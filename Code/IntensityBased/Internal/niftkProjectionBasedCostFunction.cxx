/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkProjectionBasedCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>

namespace niftk
{

//-----------------------------------------------------------------------------
ProjectionBasedCostFunction::ProjectionBasedCostFunction()
{
}


//-----------------------------------------------------------------------------
ProjectionBasedCostFunction::~ProjectionBasedCostFunction()
{
}


//-----------------------------------------------------------------------------
void ProjectionBasedCostFunction::Initialise(const cv::Size2i& windowSize, const std::string& model)
{
  if (windowSize.width <= 0)
  {
    niftkNiftyCalThrow() << "Invalid width.";
  }

  if (windowSize.height <= 0)
  {
    niftkNiftyCalThrow() << "Invalid height.";
  }

  // Load Model, and extend it.
}


//-----------------------------------------------------------------------------
unsigned int ProjectionBasedCostFunction::BiLinearInterpolate(const cv::Mat& image, cv::Point2f& pixel) const
{
  return 0;
}


//-----------------------------------------------------------------------------
void ProjectionBasedCostFunction::AccumulateSamples(const cv::Mat& greyScaleVideoImageA,
                                                    const cv::Mat& intrinsicsA,
                                                    const cv::Mat& distortionA,
                                                    const cv::Mat& rvecA,
                                                    const cv::Mat& tvecA,
                                                    const cv::Mat& greyScaleVideoImageB,
                                                    const cv::Mat& intrinsicsB,
                                                    const cv::Mat& distortionB,
                                                    const cv::Mat& rvecB,
                                                    const cv::Mat& tvecB,
                                                    unsigned long int& counter,
                                                    cv::Mat& histogramRows,
                                                    cv::Mat& histogramCols,
                                                    cv::Mat& jointHistogram
                                                   ) const
{
  if (greyScaleVideoImageA.size() != greyScaleVideoImageB.size())
  {
    niftkNiftyCalThrow() << "Images have different size.";
  }

  for (int i = 0; i < m_Model.size(); i++)
  {
    std::vector<cv::Point2f> projectedA;
    std::vector<cv::Point2f> projectedB;
    std::vector<cv::Point3f> modelPoint;
    modelPoint.push_back(m_Model[i]);

    // Just doing one at a time for now.
    cv::projectPoints(modelPoint,
                      rvecA,
                      tvecA,
                      intrinsicsA,
                      distortionA,
                      projectedA);

    cv::projectPoints(modelPoint,
                      rvecB,
                      tvecB,
                      intrinsicsB,
                      distortionB,
                      projectedB);

    // Check both points project inside images.
    if (   projectedA[0].x > 0
        && projectedA[0].x < (greyScaleVideoImageA.cols - 1)
        && projectedA[0].y > 0
        && projectedA[0].y < (greyScaleVideoImageA.rows - 1)
        && projectedB[0].x > 0
        && projectedB[0].x < (greyScaleVideoImageB.cols - 1)
        && projectedB[0].y > 0
        && projectedB[0].y < (greyScaleVideoImageB.rows - 1)
       )
    {
      unsigned int a = this->BiLinearInterpolate(greyScaleVideoImageA, projectedA[0]);
      unsigned int b = this->BiLinearInterpolate(greyScaleVideoImageB, projectedB[0]);
      jointHistogram.at<double>(a, b) += 1;
      histogramRows.at<double>(a, 0) += 1;
      histogramCols.at<double>(0, b) += 1;
      counter += 1;
    }
  }
}

} // end niftk
