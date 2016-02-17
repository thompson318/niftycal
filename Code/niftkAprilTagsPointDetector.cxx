/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkAprilTagsPointDetector.h"
#include "niftkNiftyCalExceptionMacro.h"

#include <AprilTags/TagDetector.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>

namespace niftk {

//-----------------------------------------------------------------------------
AprilTagsPointDetector::AprilTagsPointDetector(
    cv::Mat* image,
    bool includeCorners,
    const std::string& name,
    float sigma,
    float segmentationSigma
    )
: m_Image(image)
, m_IncludeCorners(includeCorners)
, m_Name(name)
, m_Sigma(sigma)
, m_SegmentationSigma(segmentationSigma)
{
  if (m_Image == nullptr)
  {
    niftkNiftyCalThrow() << "Image should not be NULL.";
  }
  if (m_Name.size() == 0)
  {
    niftkNiftyCalThrow() << "Name should not be empty.";
  }
  if (m_Sigma < 0)
  {
    niftkNiftyCalThrow() << "Sigma should not be negative.";
  }
  if (m_SegmentationSigma < 0)
  {
    niftkNiftyCalThrow() << "Segmentation sigma should not be negative.";
  }
}


//-----------------------------------------------------------------------------
AprilTagsPointDetector::~AprilTagsPointDetector()
{
  // Do NOT destroy m_Image, we don't own it.
}


//-----------------------------------------------------------------------------
PointSet AprilTagsPointDetector::GetPoints()
{
  PointSet result;

  AprilTags::TagCodes tagCodes(AprilTags::tagCodes36h11);
  if (m_Name == "16h5") {
     tagCodes = AprilTags::tagCodes16h5;
   } else if (m_Name == "25h7") {
     tagCodes = AprilTags::tagCodes25h7;
   } else if (m_Name == "25h9") {
     tagCodes = AprilTags::tagCodes25h9;
   } else if (m_Name == "36h9") {
     tagCodes = AprilTags::tagCodes36h9;
   } else if (m_Name == "36h11"){
     tagCodes = AprilTags::tagCodes36h11;
   }
   else
   {
    niftkNiftyCalThrow() << "Invalid tag name:" << m_Name;
   }

  AprilTags::TagDetector tagDetector(tagCodes);
  tagDetector.SetSigma(m_Sigma);
  tagDetector.SetSegmentationSigma(m_SegmentationSigma);
  tagDetector.SetUseHybridMethod(false);

  std::vector<AprilTags::TagDetection> markers = tagDetector.extractTags(*m_Image);

  for (unsigned int i = 0; i < markers.size(); i++)
  {
    Point2D tmp;

    tmp.point.x = (double) markers[i].cxy.first;
    tmp.point.y = (double) markers[i].cxy.second;
    tmp.id = markers[i].id;
    result.insert(IdPoint2D(tmp.id, tmp));

    if (m_IncludeCorners)
    {
      tmp.point.x = (double) markers[i].p[3].first;
      tmp.point.y = (double) markers[i].p[3].second;
      tmp.id = markers[i].id + 1000;
      result.insert(IdPoint2D(tmp.id, tmp));

      tmp.point.x = (double) markers[i].p[2].first;
      tmp.point.y = (double) markers[i].p[2].second;
      tmp.id = markers[i].id + 2000;
      result.insert(IdPoint2D(tmp.id, tmp));

      tmp.point.x = (double) markers[i].p[1].first;
      tmp.point.y = (double) markers[i].p[1].second;
      tmp.id = markers[i].id + 3000;
      result.insert(IdPoint2D(tmp.id, tmp));

      tmp.point.x = (double) markers[i].p[0].first;
      tmp.point.y = (double) markers[i].p[0].second;
      tmp.id = markers[i].id + 4000;
      result.insert(IdPoint2D(tmp.id, tmp));
    }
  }

  return result;
}

} // end namespace
