/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearNoIntrinsicsCostFunction_h
#define niftkNonLinearNoIntrinsicsCostFunction_h

#include <cv.h>

namespace niftk
{

/**
* \class NonLinearNoIntrinsicsCostFunction
* \brief Base class for cost functions that don't optimise intrinsic parameters.
*/
class NonLinearNoIntrinsicsCostFunction
{

public:

  void SetIntrinsic(const cv::Mat* const intrinsic);
  void SetDistortion(const cv::Mat* const distortion);

  void SetRightIntrinsic(const cv::Mat* const intrinsic);
  void SetRightDistortion(const cv::Mat* const distortion);

protected:

  NonLinearNoIntrinsicsCostFunction();
  virtual ~NonLinearNoIntrinsicsCostFunction();

  NonLinearNoIntrinsicsCostFunction(const NonLinearNoIntrinsicsCostFunction&);
  NonLinearNoIntrinsicsCostFunction& operator=(const NonLinearNoIntrinsicsCostFunction&);

  cv::Mat* m_Intrinsic;
  cv::Mat* m_Distortion;
  cv::Mat* m_RightIntrinsic;
  cv::Mat* m_RightDistortion;
};

} // end namespace

#endif
