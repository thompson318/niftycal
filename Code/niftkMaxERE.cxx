/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkMaxERE.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk {

//-----------------------------------------------------------------------------
double ComputeMaxEREFromCalibrations(
    const std::vector<cv::Point3f>& testPointsXYZ,
    const std::pair<cv::Mat, cv::Mat>& x,
    const std::vector< std::pair<cv::Mat, cv::Mat> >& calSamples
    )
{
  double MaxERE = 0;
  double squaredDiff= 0;

  for (size_t i = 0; i < testPointsXYZ.size(); i++)
  {
    cv::Point3d testPoint = testPointsXYZ[i];
    std::vector< cv::Point3f > tmpInput;
    tmpInput.push_back(testPoint);

    std::vector< cv::Point2f > projectedViaMeanCalibration;
    cv::projectPoints(tmpInput, cv::Mat(), cv::Mat(), x.first, x.second, projectedViaMeanCalibration);

    double ERE = 0;
    size_t counter = 0;
    std::vector< cv::Point2f > projectedViaSampleCalibration;

    for (size_t j = 0; j < calSamples.size(); j++)
    {
      cv::projectPoints(tmpInput, cv::Mat(), cv::Mat(), calSamples[i].first, calSamples[i].second, projectedViaSampleCalibration);
      squaredDiff = (projectedViaMeanCalibration[0].x - projectedViaSampleCalibration[0].x)
                  * (projectedViaMeanCalibration[0].x - projectedViaSampleCalibration[0].x)
                  + (projectedViaMeanCalibration[0].y - projectedViaSampleCalibration[0].y)
                  * (projectedViaMeanCalibration[0].y - projectedViaSampleCalibration[0].y);
      ERE += sqrt(squaredDiff);
      counter++;
    }
    if (counter > 0)
    {
      ERE = ERE/static_cast<double>(counter);
    }
    MaxERE = std::max(MaxERE, ERE);
  }
  return MaxERE;
}


//-----------------------------------------------------------------------------
void GetModelPosterior(
    const std::pair<cv::Mat, cv::Mat>& currentCalibration,
    std::pair<cv::Mat, cv::Mat>& x,
    std::vector< std::pair<cv::Mat, cv::Mat> >& calSamples
    )
{
  niftkNiftyCalThrow() << "Not implemented yet.";
}


//-----------------------------------------------------------------------------
double ComputeMaxERE(
    const cv::Mat& intrinsic,
    const cv::Mat& distortion
    )
{

  std::vector<cv::Point3f> testPointsXYZ;

  std::pair<cv::Mat, cv::Mat> currentCalib = std::pair<cv::Mat, cv::Mat>(intrinsic, distortion);
  std::pair<cv::Mat, cv::Mat> meanCalib = std::pair<cv::Mat, cv::Mat>(intrinsic.clone(), distortion.clone());

  std::vector< std::pair<cv::Mat, cv::Mat> > calibrationSamples;
  niftk::GetModelPosterior(currentCalib, meanCalib, calibrationSamples);

  return ComputeMaxEREFromCalibrations(testPointsXYZ, meanCalib, calibrationSamples);
}

} // end namespace
