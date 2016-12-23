/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkPoseFromPoints.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk
{

//-----------------------------------------------------------------------------
void PoseFromPoints(const Model3D& model,
                    const std::list<PointSet>& listOfPointSets,
                    cv::Mat& intrinsic,
                    cv::Mat& distortion,
                    std::vector<cv::Mat>& rvecs,
                    std::vector<cv::Mat>& tvecs
                   )
{
  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model is empty.";
  }

  rvecs.clear();
  tvecs.clear();

  unsigned int viewCounter = 0;

  std::list<PointSet>::const_iterator listIter;
  PointSet::const_iterator pointsIter;

  for (listIter = listOfPointSets.begin(); listIter != listOfPointSets.end(); ++listIter)
  {
    std::vector<cv::Vec3f> vectors3D;
    std::vector<cv::Vec2f> vectors2D;

    for (pointsIter = (*listIter).begin(); pointsIter != (*listIter).end(); ++pointsIter)
    {
      // i.e. the 2D point ID is contained within the model.
      if (model.find((*pointsIter).first) != model.end())
      {
        cv::Point3d p3 = (model.at((*pointsIter).first)).point;
        cv::Vec3f v3(p3.x, p3.y, p3.z);

        cv::Point2d p2 = ((*pointsIter).second).point;
        cv::Vec2f v2(p2.x, p2.y);

        vectors3D.push_back(v3);
        vectors2D.push_back(v2);
      }
    }

    if (vectors3D.size() >= 4 && vectors2D.size() >= 4 && vectors3D.size() == vectors2D.size())
    {
      cv::Mat rvec;
      cv::Mat tvec;
      cv::solvePnP(vectors3D,
                   vectors2D,
                   intrinsic,
                   distortion,
                   rvec,
                   tvec
                  );

      rvecs.push_back(rvec);
      tvecs.push_back(tvec);

    }
    else
    {
      std::cout << "Warning: Dropping view " << viewCounter << ", as there were < 4 points." << std::endl;
    }
    viewCounter++;
  }

  // Sanity check
  if (rvecs.size() == 0)
  {
    niftkNiftyCalThrow() << "No poses found";
  }

 return;
}

} // end namespace
