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

}

} // end niftk
