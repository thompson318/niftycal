/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkOpenCVPointDetector_h
#define niftkOpenCVPointDetector_h

#include "niftkWin32ExportHeader.h"
#include <niftkIPoint2DDetector.h>
#include <cv.h>

namespace niftk
{

/**
* \class OpenCVPointDetector
* \brief Base class for OpenCV based IPoint2DDetectors, mainly to handle image
* rescaling, and the resultant scaling of the detected points.
*
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT OpenCVPointDetector : public IPoint2DDetector
{

public:

  OpenCVPointDetector();
  virtual ~OpenCVPointDetector();

  /**
  * \brief Gives this dector a pointer to an image,
  * (remember to watch out for OpenCV reference counting).
  */
  void SetImage(cv::Mat* image);

  /**
  * \brief Enables you to set a multiplicative scale up/down factor,
  * e.g. your image is 1920x540, and you need 1920x1080,
  * so you can set cv::Point2d(1,2).
  */
  void SetImageScaleFactor(const cv::Point2d& scaleFactor);

  /**
  * \brief Implements IPoint2DDetector::GetPoints()
  * and derived classes should reimplement InternalGetPoints().
  */
  virtual PointSet GetPoints();

protected:

  /**
  * \brief Derived classes should implement this to extract points.
  */
  virtual PointSet InternalGetPoints(const cv::Mat& imageToUse) = 0;

private:
  cv::Mat*    m_Image;
  cv::Mat     m_RescaledImage;
  cv::Point2d m_ScaleFactors;
};

} // end namespace

#endif