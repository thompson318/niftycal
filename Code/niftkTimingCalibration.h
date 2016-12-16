/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkTimingCalibration_h
#define niftkTimingCalibration_h

#include "niftkWin32ExportHeader.h"
#include "niftkNiftyCalTypes.h"

namespace niftk
{

/**
* \file niftkTimingCalibration.h
* \brief Timing calibration routines.
* \ingroup calibration
*/


/**
* \brief For two samples of the same motion, calculates time difference between a and b.
* \param a Fastest data set, e.g. Tracker at 60fps
* \param b Slowest data set, e.g. Video at 20fps
* \return lag between b and a in milliseconds, where a positive number indicates b is behind a.
*/
NIFTYCAL_WINEXPORT int TimingCalibration(const TimeSamples1D& a,
                                         const TimeSamples1D& b
                                        );


/**
* \brief Overriden method, taking 3D points, for example from a tracker translation matrix
* and 2D points taken from some feature extraction in an image.
*
* This method just converts b to a 3D sequence, and calls the other TimingCalibration method.
*/
NIFTYCAL_WINEXPORT int TimingCalibration(const TimeSamples3D& a,
                                         const TimeSamples2D& b
                                        );


/**
* \brief Overriden method, taking two sets of 3D points.
*
* This method just does PCA to find major axis of both datasets,
* projects points onto that axis, and thereby creates a 1D signal,
* and calls the other TimingCalibration method.
*/
NIFTYCAL_WINEXPORT int TimingCalibration(const TimeSamples3D& a,
                                         const TimeSamples3D& b
                                        );


/**
* \brief Utility function to project a 3D dataset to 1D via PCA, and normalise to [0,1].
*/
NIFTYCAL_WINEXPORT TimeSamples1D ProjectTo1DAndNormalise(const TimeSamples3D& a);


/**
* \brief Utility function to linearly interpolate time sequence every
* millisecond, at timepoints exactly on millimetre boundaries (no fractions of milliseconds).
*/
NIFTYCAL_WINEXPORT TimeMappedSamples1D ResampleTimeStampsToMilliseconds(const TimeSamples1D& a);


/**
* \brief Computes Normalised Cross Correlation of two time series, given an offset, applied to b.
*
* Assumes a and b are ordered, valid timestamps and do not have fractions of milliseconds.
*/
NIFTYCAL_WINEXPORT double ComputeNCC(const TimeMappedSamples1D& a, const TimeMappedSamples1D& b, const int& offsetInMilliseconds);

} // end namespace

#endif
