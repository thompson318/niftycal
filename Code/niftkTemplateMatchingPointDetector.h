/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkTemplateMatchingPointDetector_h
#define niftkTemplateMatchingPointDetector_h

#include "niftkWin32ExportHeader.h"
#include "niftkPointDetector.h"

namespace niftk
{

/**
* \class TemplateMatchingPointDetector
* \brief Base class for template matching, as seen in Dutta 2009
* http://dx.doi.org/10.1109/ICCVW.2009.5457474
*
* This detector is not thread safe.
*/
class NIFTYCAL_WINEXPORT TemplateMatchingPointDetector : public PointDetector
{

public:

  TemplateMatchingPointDetector(cv::Size2i patternSize,      // how many rings/dots in x,y.
                                cv::Size2i offsetForTemplate // how many pixels to search in x,y.
                               );
  virtual ~TemplateMatchingPointDetector();

  void SetReferenceImage(cv::Mat* image);
  void SetTemplateImage(cv::Mat* image);
  void SetReferencePoints(const niftk::PointSet& points);

  void SetMaxAreaInPixels(unsigned long int& pixels);
  void SetUseContours(bool useContours);
  void SetUseInternalResampling(bool useResampling);
  void SetUseTemplateMatching(bool useTemplateMatching);

protected:

  /**
  * \see niftk::PointDetector::InternalGetPoints()
  */
  virtual PointSet InternalGetPoints(const cv::Mat& imageToUse);

  /**
  * \brief Coordinates the template matching and resampling process.
  */
  virtual PointSet GetPointsUsingTemplateMatching(const cv::Mat& image, const niftk::PointSet& startingGuess);

  /**
  * \brief Derived classes must implement a quick contour based method to extract initial guesses.
  */
  virtual PointSet GetPointsUsingContours(const cv::Mat& image) = 0;

  cv::Size2i        m_PatternSize;
  cv::Size2i        m_OffsetForTemplate;
  unsigned long int m_MaxAreaInPixels;
  bool              m_UseContours;
  bool              m_UseInternalResampling;
  bool              m_UseTemplateMatching;
  niftk::PointSet   m_ReferencePoints;
  cv::Mat*          m_ReferenceImage;
  cv::Mat*          m_TemplateImage;
};

} // end namespace

#endif
