/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkTimingCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-----------------------------------------------------------------------------
TimeSamples1D ProjectTo1DAndNormalise(const TimeSamples3D& a)
{
  TimeSamples1D result;

  size_t numberSamples = static_cast<size_t>(a.size());

  cv::Mat data = cvCreateMat(numberSamples, 3, CV_64FC1);

  size_t sampleCounter = 0;
  TimeSamples3D::const_iterator iter;
  for (iter = a.begin(); iter != a.end(); ++iter)
  {
    data.at<double>(sampleCounter, 0) = (*iter).sample.x;
    data.at<double>(sampleCounter, 1) = (*iter).sample.y;
    data.at<double>(sampleCounter, 2) = (*iter).sample.z;
    sampleCounter++;
  }

  cv::PCA pca(data, cv::Mat(), CV_PCA_DATA_AS_ROW);

  cv::Point3d centroid;
  centroid.x = pca.mean.at<double>(0, 0);
  centroid.y = pca.mean.at<double>(0, 1);
  centroid.z = pca.mean.at<double>(0, 2);

  cv::Point3d majorVector;
  majorVector.x = pca.eigenvectors.at<double>(0, 0);
  majorVector.y = pca.eigenvectors.at<double>(0, 1);
  majorVector.z = pca.eigenvectors.at<double>(0, 2);

  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::min();
  TimingSample1D tmp;

  for (iter = a.begin(); iter != a.end(); ++iter)
  {
    tmp.time = (*iter).time;
    tmp.sample = ((*iter).sample.x - centroid.x) * majorVector.x // dot product, with major axis.
               + ((*iter).sample.y - centroid.y) * majorVector.y // gives length (i.e. projection).
               + ((*iter).sample.z - centroid.z) * majorVector.z;

    if (tmp.sample < min)
    {
      min = tmp.sample;
    }
    else if (tmp.sample > max)
    {
      max = tmp.sample;
    }
    result.push_back(tmp);
  }

  // Normalise to [0,1].
  double range = max - min;
  TimeSamples1D::iterator resultIter;
  for (resultIter = result.begin(); resultIter != result.end(); ++resultIter)
  {
    (*resultIter).sample = ((*resultIter).sample - min) / range;
  }

  return result;
}


//-----------------------------------------------------------------------------
TimeMappedSamples1D ResampleTimeStampsToMilliseconds(const TimeSamples1D& a)
{
  TimeMappedSamples1D result;

  TimingSample1D current;
  TimingSample1D previous;
  TimingSample1D newSample;

  NiftyCalTimeType nanosToMillis = 1000000;
  NiftyCalTimeType t;

  for (size_t i = 1; i < a.size(); i++)
  {
    current  = a[i];
    previous = a[i-1];

    NiftyCalTimeType roundedCurrentTime = (current.time / nanosToMillis) * nanosToMillis;
    NiftyCalTimeType roundedPreviousTime = (previous.time / nanosToMillis) * nanosToMillis;

    if (roundedPreviousTime == roundedCurrentTime)
    {
      niftkNiftyCalThrow() << "Timestamps are too close.";
    }
    if (roundedCurrentTime < roundedPreviousTime)
    {
      niftkNiftyCalThrow() << "Timestamps are incorrectly ordered.";
    }

    NiftyCalTimeType timeGap = roundedCurrentTime - roundedPreviousTime;
    double sampleGap = current.sample - previous.sample;

    for (t = roundedPreviousTime; t <= roundedCurrentTime; t += nanosToMillis)
    {
      newSample.time = t;

      // linear interpolation.
      newSample.sample = previous.sample + (static_cast<double>(t - roundedPreviousTime) * sampleGap
                                           / static_cast<double>(timeGap));

      result.insert(std::pair<NiftyCalTimeType, double>(newSample.time, newSample.sample));
    }
  }
  return result;
}


//-----------------------------------------------------------------------------
double ComputeNCC(const TimeMappedSamples1D& a, const TimeMappedSamples1D& b, const int& offsetInMilliseconds)
{
  double cost = 0;

  // We don't want the min and max of the union.
  // We want the min and max of the intersection.
  // So this looks wrong, but its intentionally swapped round.
  NiftyCalTimeType min = std::max(a.begin()->first, b.begin()->first);
  NiftyCalTimeType max = std::min(a.rbegin()->first, b.rbegin()->first);

  if (max == 0)
  {
    niftkNiftyCalThrow() << "max timestamp is zero.";
  }
  if (min == 0)
  {
    niftkNiftyCalThrow() << "min timestamp is zero.";
  }
  if (max < min)
  {
    niftkNiftyCalThrow() << "max timestamp is earlier than min timestamp.";
  }

  TimeMappedSamples1D::const_iterator aIter;
  TimeMappedSamples1D::const_iterator bIter;
  double sumAA = 0;
  double sumBB = 0;
  double sumAB = 0;
  double sumA = 0;
  double sumB = 0;
  double aValue = 0;
  double bValue = 0;
  int millisToNanos = 1000000;

  TimeMappedSamples1D::size_type numberSamples = 0;

  for (NiftyCalTimeType t = min; t <= max; t+= millisToNanos)
  {
    aIter = a.find(t);
    bIter = b.find(t - (offsetInMilliseconds*millisToNanos));
    if (aIter != a.end() && bIter != b.end())
    {
      aValue = (*aIter).second;
      bValue = (*bIter).second;
      sumAA += aValue*aValue;
      sumBB += bValue*bValue;
      sumAB += aValue*bValue;
      sumA += aValue;
      sumB += bValue;
      numberSamples++;
    }
  }

  if (numberSamples == 0)
  {
    niftkNiftyCalThrow() << "No samples for NCC, which suggests timestamps are wrong, or wrongly resampled.";
  }

  if (numberSamples > 0)
  {
    sumAA -= ((sumA * sumA) / static_cast<double>(numberSamples));
    sumBB -= ((sumB * sumB) / static_cast<double>(numberSamples));
    sumAB -= ((sumA * sumB) / static_cast<double>(numberSamples));
  }

  double denom = std::sqrt(sumAA * sumBB);

  if (numberSamples > 0 && denom != 0)
  {
    cost = sumAB / denom;
  }

  return cost;
}


//-----------------------------------------------------------------------------
int TimingCalibration(const TimeSamples3D& a,
                      const TimeSamples2D& b
                     )
{
  TimeSamples3D bIn3D;
  TimeSamples2D::const_iterator iter;
  TimingSample3D tmp;
  tmp.sample.z = 0;

  for (iter = b.begin(); iter != b.end(); ++iter)
  {
    tmp.time = (*iter).time;
    tmp.sample.x = (*iter).sample.x;
    tmp.sample.y = (*iter).sample.y;
    bIn3D.push_back(tmp);
  }
  return TimingCalibration(a, bIn3D);
}


//-----------------------------------------------------------------------------
int TimingCalibration(const TimeSamples3D& a,
                      const TimeSamples3D& b
                     )
{
  TimeSamples1D aIn1D = ProjectTo1DAndNormalise(a);
  TimeSamples1D bIn1D = ProjectTo1DAndNormalise(b);
  return TimingCalibration(aIn1D, bIn1D);
}


//-----------------------------------------------------------------------------
int TimingCalibration(const TimeSamples1D& a,
                      const TimeSamples1D& b
                     )
{

  if (a.size() < 2)
  {
    niftkNiftyCalThrow() << "a should have 2 or more items.";
  }
  if (b.size() < 2)
  {
    niftkNiftyCalThrow() << "b should have 2 or more items.";
  }

  int offset = 0;
  int bestOffset = 0;
  double bestCost = 0;

  // This also converts to a hash map for the subsequent optimisation.
  TimeMappedSamples1D aMillis = ResampleTimeStampsToMilliseconds(a);
  TimeMappedSamples1D bMillis = ResampleTimeStampsToMilliseconds(b);

  // Work out which direction to move in, using forward difference.
  double multiplier = 1.0;

  double c0 = ComputeNCC(aMillis, bMillis, 0);
  double c1 = ComputeNCC(aMillis, bMillis, 1);

  if (c0 < 0 && c1 < 0)
  {
    // time signals are negatively correlated.
    // e.g one signal going up-down while the other is down-up.
    multiplier = -1.0;
    c0 *= multiplier;
    c1 *= multiplier;
  }

  int step = 1;
  if (c1 < c0)
  {
    step = -1;
  }

  double previousCost = std::numeric_limits<double>::min();
  double currentCost = c0;

  // Just keep walking uphill, 1ms at a time, until we go over the peak.
  // Assumes higher value is better, so cost function should be something like NCC.
  while (currentCost > previousCost)
  {
    bestOffset = offset;
    bestCost = currentCost;
    offset += step;
    previousCost = currentCost;
    currentCost = ComputeNCC(aMillis, bMillis, offset) * multiplier;
    std::cout << "TimingCalibration[" << offset << "]=" << currentCost << std::endl;
  }
  std::cout << "TimingCalibration[" << bestOffset << "]=" << bestCost << std::endl;

  return -bestOffset;
}

} // end namespace
