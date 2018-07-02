/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/
#include "niftkStereoCameraCalibration.h"
#include "niftkZhangCameraCalibration.h"
#include "niftkTsaiCameraCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkPointUtilities.h"
#include "niftkMatrixUtilities.h"
#include <Internal/niftkCalibrationUtilities_p.h>
#include <fstream>
#include <list>
#include <algorithm>

#ifdef NIFTYCAL_WITH_ITK
#include <Internal/niftkNonLinearStereoCameraCalibration2DOptimiser.h>
#include <Internal/niftkNonLinearStereoIntrinsicsCalibration3DOptimiser.h>
#include <Internal/niftkNonLinearStereoExtrinsicsCalibration3DOptimiser.h>
#include <Internal/niftkNonLinearMonoCameraCalibration3DOptimiser.h>
#endif

#include <Internal/niftkNonLinearCeresStereoOptimiser.h>
#include <Internal/niftkNonLinearCeresMonoOptimiser.h>

#include <highgui.h>

namespace niftk
{

//-----------------------------------------------------------------------------
cv::Matx21d StereoCameraCalibration(const Model3D& model,
                                    const std::list<PointSet>& listOfLeftHandPointSets,
                                    const std::list<PointSet>& listOfRightHandPointSets,
                                    const cv::Size2i& imageSize,
                                    cv::Mat& intrinsicLeft,
                                    cv::Mat& distortionLeft,
                                    std::vector<cv::Mat>& rvecsLeft,
                                    std::vector<cv::Mat>& tvecsLeft,
                                    cv::Mat& intrinsicRight,
                                    cv::Mat& distortionRight,
                                    std::vector<cv::Mat>& rvecsRight,
                                    std::vector<cv::Mat>& tvecsRight,
                                    cv::Mat& leftToRightRotationMatrix,
                                    cv::Mat& leftToRightTranslationVector,
                                    cv::Mat& essentialMatrix,
                                    cv::Mat& fundamentalMatrix,
                                    const int& cvFlags,
                                    const bool& optimise3D
                                   )
{
  cv::Matx21d result;
  result(0, 0) = 0;
  result(1, 0) = 0;

  double projectedRMS = 0;
  double reconstructedRMS = 0;
  cv::Point3d rmsInEachAxis;

  if (model.empty())
  {
    niftkNiftyCalThrow() << "Model is empty.";
  }
  if (listOfLeftHandPointSets.empty())
  {
    niftkNiftyCalThrow() << "Should have at least 1 view of calibration points for left camera.";
  }
  if (listOfRightHandPointSets.empty())
  {
    niftkNiftyCalThrow() << "Should have at least 1 view of calibration points for right camera.";
  }
  if (listOfLeftHandPointSets.size() != listOfRightHandPointSets.size())
  {
    niftkNiftyCalThrow() << "Should have the same number of views in left and right channel.";
  }

  unsigned int viewCounter = 0;
  std::list<PointSet>::const_iterator iter;

  for (iter = listOfLeftHandPointSets.begin(); iter != listOfLeftHandPointSets.end(); ++iter)
  {
    if ((*iter).size() < 4)
    {
      niftkNiftyCalThrow() << "Should have 4 or more points in the " << viewCounter << "th left camera view.";
    }
    viewCounter++;
  }
  viewCounter = 0;
  for (iter = listOfRightHandPointSets.begin(); iter != listOfRightHandPointSets.end(); ++iter)
  {
    if ((*iter).size() < 4)
    {
      niftkNiftyCalThrow() << "Should have 4 or more points in the " << viewCounter << "th right camera view.";
    }
    viewCounter++;
  }

  std::vector<std::vector<cv::Vec3f> > objectPoints;
  std::vector<std::vector<cv::Vec2f> > leftImagePoints;
  std::vector<std::vector<cv::Vec2f> > rightImagePoints;

  // Remember at this point, the number of PointSets in left and right is the same.

  std::list<PointSet>::const_iterator leftListIter;
  std::list<PointSet>::const_iterator rightListIter;
  PointSet::const_iterator leftPointsIter;
  PointSet::const_iterator rightPointsIter;

  // Loop through each point set.
  viewCounter = 0;
  for (leftListIter = listOfLeftHandPointSets.begin(),
       rightListIter = listOfRightHandPointSets.begin();
       leftListIter != listOfLeftHandPointSets.end() &&
       rightListIter != listOfRightHandPointSets.end();
       ++leftListIter,
       ++rightListIter
       )
  {
    std::vector<cv::Vec3f> objectVectors3D;
    std::vector<cv::Vec2f> leftVectors2D;
    std::vector<cv::Vec2f> rightVectors2D;

    // A point must be visible in both left and right view for it to be added.
    for (leftPointsIter = (*leftListIter).begin(); leftPointsIter != (*leftListIter).end(); ++leftPointsIter)
    {
      niftk::NiftyCalIdType id = (*leftPointsIter).first;
      rightPointsIter = (*rightListIter).find(id);

      if (rightPointsIter != (*rightListIter).end())
      {
        // Found matching right hand point
        cv::Point3d p3 = (model.at((*leftPointsIter).first)).point;
        cv::Vec3f v3(p3.x, p3.y, p3.z);

        cv::Point2d p2l = ((*leftPointsIter).second).point;
        cv::Vec2f v2l(p2l.x, p2l.y);

        cv::Point2d p2r = ((*rightPointsIter).second).point;
        cv::Vec2f v2r(p2r.x, p2r.y);

        objectVectors3D.push_back(v3);
        leftVectors2D.push_back(v2l);
        rightVectors2D.push_back(v2r);
      }
    }

    if (objectVectors3D.size() >= 4 && leftVectors2D.size() >= 4 && rightVectors2D.size() >= 4
        && objectVectors3D.size() == leftVectors2D.size()
        && objectVectors3D.size() == rightVectors2D.size()
        )
    {
      objectPoints.push_back(objectVectors3D);
      leftImagePoints.push_back(leftVectors2D);
      rightImagePoints.push_back(rightVectors2D);
    }
    else
    {
      std::cout << "Warning: Dropping view " << viewCounter
                << ", as there were < 4 common points across both views." << std::endl;
    }
    viewCounter++;
  }

  // Sanity check
  if (objectPoints.size() == 0)
  {
    niftkNiftyCalThrow() << "No object points extracted.";
  }
  if (leftImagePoints.size() == 0)
  {
    niftkNiftyCalThrow() << "No left image points extracted.";
  }
  if (rightImagePoints.size() == 0)
  {
    niftkNiftyCalThrow() << "No right image points extracted.";
  }

  // Do standard OpenCV calibration
  projectedRMS = cv::stereoCalibrate(objectPoints,
                                     leftImagePoints,
                                     rightImagePoints,
                                     intrinsicLeft,
                                     distortionLeft,
                                     intrinsicRight,
                                     distortionRight,
                                     imageSize,
                                     leftToRightRotationMatrix,
                                     leftToRightTranslationVector,
                                     essentialMatrix,
                                     fundamentalMatrix,
                                     cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 10000, 1e-10),
                                     cvFlags
                                    );

  std::list<PointSet>::const_iterator leftIter;
  std::list<PointSet>::const_iterator rightIter;
  unsigned int v = 0;
  for (leftIter = listOfLeftHandPointSets.begin(), rightIter = listOfRightHandPointSets.begin();
       leftIter != listOfLeftHandPointSets.end() && rightIter != listOfRightHandPointSets.end();
       leftIter++, rightIter++)
  {
    std::ostringstream fileName;
    fileName << "/Users/mattclarkson/calib.opencv.left." << v << ".png";
    cv::Matx44d extrinsic = niftk::RodriguesToMatrix(rvecsLeft[v], tvecsLeft[v]);
    cv::Mat image = niftk::ProjectPointsToImage(model, *leftIter, extrinsic, intrinsicLeft, distortionLeft, cv::Size(1920, 540));
    cv::imwrite(fileName.str(), image);

    std::ostringstream fileName2;
    fileName2 << "/Users/mattclarkson/calib.opencv.right." << v << ".png";
    extrinsic = niftk::RodriguesToMatrix(rvecsRight[v], tvecsRight[v]);
    image = niftk::ProjectPointsToImage(model, *rightIter, extrinsic, intrinsicRight, distortionRight, cv::Size(1920, 540));
    cv::imwrite(fileName2.str(), image);
    v++;
  }

  std::cout << "niftkStereoCameraCalibration:OpenCV optimisation finished, cvFlags=" << cvFlags << ", rms2D=" << projectedRMS << std::endl;

  /*
  // Recompute left-to-right using rigid body registration.
  double fre = ComputeLeftToRight(model,
                                  rvecsLeft, tvecsLeft,
                                  rvecsRight, tvecsRight,
                                  leftToRightRotationMatrix, leftToRightTranslationVector
                                 );
  */

  // Ensure rhs extrinsics are set from lhs and left-to-right transform.
  niftk::ComputeStereoExtrinsics(rvecsLeft,
                                 tvecsLeft,
                                 leftToRightRotationMatrix,
                                 leftToRightTranslationVector,
                                 rvecsRight,
                                 tvecsRight
                                );

  // Recompute re-projection error, now we have updated the right extrinsics.
  projectedRMS = niftk::ComputeRMSReprojectionError(model,
                                                    listOfLeftHandPointSets,
                                                    listOfRightHandPointSets,
                                                    intrinsicLeft,
                                                    distortionLeft,
                                                    rvecsLeft,
                                                    tvecsLeft,
                                                    intrinsicRight,
                                                    distortionRight,
                                                    leftToRightRotationMatrix,
                                                    leftToRightTranslationVector
                                                   );

  // Compute 3D reconstruction error, now we have updated the right extrinsics.
  reconstructedRMS = niftk::ComputeRMSReconstructionError(model,
                                                          listOfLeftHandPointSets,
                                                          listOfRightHandPointSets,
                                                          intrinsicLeft,
                                                          distortionLeft,
                                                          rvecsLeft,
                                                          tvecsLeft,
                                                          intrinsicRight,
                                                          distortionRight,
                                                          leftToRightRotationMatrix,
                                                          leftToRightTranslationVector,
                                                          rmsInEachAxis
                                                         );


  v = 0;
  for (leftIter = listOfLeftHandPointSets.begin(), rightIter = listOfRightHandPointSets.begin();
       leftIter != listOfLeftHandPointSets.end() && rightIter != listOfRightHandPointSets.end();
       leftIter++, rightIter++)
  {
    std::ostringstream fileName;
    fileName << "/Users/mattclarkson/calib.reset.left." << v << ".png";
    cv::Matx44d extrinsic = niftk::RodriguesToMatrix(rvecsLeft[v], tvecsLeft[v]);
    cv::Mat image = niftk::ProjectPointsToImage(model, *leftIter, extrinsic, intrinsicLeft, distortionLeft, cv::Size(1920, 540));
    cv::imwrite(fileName.str(), image);

    std::ostringstream fileName2;
    fileName2 << "/Users/mattclarkson/calib.reset.right." << v << ".png";
    extrinsic = niftk::RodriguesToMatrix(rvecsRight[v], tvecsRight[v]);
    image = niftk::ProjectPointsToImage(model, *rightIter, extrinsic, intrinsicRight, distortionRight, cv::Size(1920, 540));
    cv::imwrite(fileName2.str(), image);
    v++;
  }

  std::cout << "niftkStereoCameraCalibration:OpenCV then reset, with recomputed metrics, rms2D=" << projectedRMS
            << ", rms3D=" << reconstructedRMS << std::endl;

#ifdef NIFTYCAL_WITH_ITK

  Model3D* tmpModel = const_cast<Model3D*>(&model);
/*
  niftk::NonLinearMonoCameraCalibration3DOptimiser::Pointer monoLeftOptimiser
      = niftk::NonLinearMonoCameraCalibration3DOptimiser::New();
  monoLeftOptimiser->SetModel(tmpModel);
  monoLeftOptimiser->SetPoints(&listOfLeftHandPointSets);
  monoLeftOptimiser->SetIntrinsic(intrinsicLeft);
  monoLeftOptimiser->SetDistortion(distortionLeft);

  double leftRMS = monoLeftOptimiser->Optimise(rvecsLeft, tvecsLeft);

  niftk::NonLinearMonoCameraCalibration3DOptimiser::Pointer monoRightOptimiser
      = niftk::NonLinearMonoCameraCalibration3DOptimiser::New();
  monoRightOptimiser->SetModel(tmpModel);
  monoRightOptimiser->SetPoints(&listOfRightHandPointSets);
  monoRightOptimiser->SetIntrinsic(intrinsicRight);
  monoRightOptimiser->SetDistortion(distortionRight);

  double rightRMS = monoRightOptimiser->Optimise(rvecsRight, tvecsRight);

  double fre2 = ComputeLeftToRight(model,
                                  rvecsLeft, tvecsLeft,
                                  rvecsRight, tvecsRight,
                                  leftToRightRotationMatrix, leftToRightTranslationVector
                                 );


  std::cout << "niftkStereoCameraCalibration::Registered l2r using ITK, FRE=" << fre
            << ", fre2=" << fre2
            << ", leftRMS=" << leftRMS
            << ", rightRMS=" << rightRMS
            << std::endl;
  */

  //double leftRMS = niftk::CeresMonoCameraCalibration(objectPoints, leftImagePoints, intrinsicLeft, distortionLeft, rvecsLeft, tvecsLeft);
  //double rightRMS = niftk::CeresMonoCameraCalibration(objectPoints, rightImagePoints, intrinsicRight, distortionRight, rvecsRight, tvecsRight);

  /*
  double fre2 = ComputeLeftToRight(model,
                                   rvecsLeft, tvecsLeft,
                                   rvecsRight, tvecsRight,
                                   leftToRightRotationMatrix, leftToRightTranslationVector
                                  );
  */
/*
  std::cout << "niftkStereoCameraCalibration::Ceres mono, leftRMS=" << leftRMS
            << ", rightRMS=" << rightRMS
            << std::endl;
            */
/*
  // Recompute re-projection error, now we have updated the right extrinsics.
  projectedRMS = niftk::ComputeRMSReprojectionError(model,
                                                    listOfLeftHandPointSets,
                                                    listOfRightHandPointSets,
                                                    intrinsicLeft,
                                                    distortionLeft,
                                                    rvecsLeft,
                                                    tvecsLeft,
                                                    intrinsicRight,
                                                    distortionRight,
                                                    leftToRightRotationMatrix,
                                                    leftToRightTranslationVector
                                                   );

  // Compute 3D reconstruction error, now we have updated the right extrinsics.
  reconstructedRMS = niftk::ComputeRMSReconstructionError(model,
                                                          listOfLeftHandPointSets,
                                                          listOfRightHandPointSets,
                                                          intrinsicLeft,
                                                          distortionLeft,
                                                          rvecsLeft,
                                                          tvecsLeft,
                                                          intrinsicRight,
                                                          distortionRight,
                                                          leftToRightRotationMatrix,
                                                          leftToRightTranslationVector,
                                                          rmsInEachAxis
                                                         );

  std::cout << "niftkStereoCameraCalibration:Ceres immediately after optimising with recomputed metrics, rms2D=" << projectedRMS
            << ", rms3D=" << reconstructedRMS << std::endl;
*/

  // Now do stereo optimisation based on Ceres.
  niftk::CeresStereoCameraCalibration(objectPoints,
                                      leftImagePoints,
                                      rightImagePoints,
                                      intrinsicLeft,
                                      distortionLeft,
                                      rvecsLeft,
                                      tvecsLeft,
                                      intrinsicRight,
                                      distortionRight,
                                      leftToRightRotationMatrix,
                                      leftToRightTranslationVector
                                     );

  // Ensure rhs extrinsics are set from lhs and left-to-right transform.
  niftk::ComputeStereoExtrinsics(rvecsLeft,
                                 tvecsLeft,
                                 leftToRightRotationMatrix,
                                 leftToRightTranslationVector,
                                 rvecsRight,
                                 tvecsRight
                                );

  // Recompute re-projection error, now we have updated the right extrinsics.
  projectedRMS = niftk::ComputeRMSReprojectionError(model,
                                                    listOfLeftHandPointSets,
                                                    listOfRightHandPointSets,
                                                    intrinsicLeft,
                                                    distortionLeft,
                                                    rvecsLeft,
                                                    tvecsLeft,
                                                    intrinsicRight,
                                                    distortionRight,
                                                    leftToRightRotationMatrix,
                                                    leftToRightTranslationVector
                                                   );

  // Compute 3D reconstruction error, now we have updated the right extrinsics.
  reconstructedRMS = niftk::ComputeRMSReconstructionError(model,
                                                          listOfLeftHandPointSets,
                                                          listOfRightHandPointSets,
                                                          intrinsicLeft,
                                                          distortionLeft,
                                                          rvecsLeft,
                                                          tvecsLeft,
                                                          intrinsicRight,
                                                          distortionRight,
                                                          leftToRightRotationMatrix,
                                                          leftToRightTranslationVector,
                                                          rmsInEachAxis
                                                         );

  v = 0;
  for (leftIter = listOfLeftHandPointSets.begin(), rightIter = listOfRightHandPointSets.begin();
       leftIter != listOfLeftHandPointSets.end() && rightIter != listOfRightHandPointSets.end();
       leftIter++, rightIter++)
  {
    std::ostringstream fileName;
    fileName << "/Users/mattclarkson/calib.ceres.left." << v << ".png";
    cv::Matx44d extrinsic = niftk::RodriguesToMatrix(rvecsLeft[v], tvecsLeft[v]);
    cv::Mat image = niftk::ProjectPointsToImage(model, *leftIter, extrinsic, intrinsicLeft, distortionLeft, cv::Size(1920, 540));
    cv::imwrite(fileName.str(), image);

    std::ostringstream fileName2;
    fileName2 << "/Users/mattclarkson/calib.ceres.right." << v << ".png";
    extrinsic = niftk::RodriguesToMatrix(rvecsRight[v], tvecsRight[v]);
    image = niftk::ProjectPointsToImage(model, *rightIter, extrinsic, intrinsicRight, distortionRight, cv::Size(1920, 540));
    cv::imwrite(fileName2.str(), image);
    v++;
  }

  std::cout << "niftkStereoCameraCalibration:Ceres 3D with recomputed metrics, rms2D=" << projectedRMS
            << ", rms3D=" << reconstructedRMS << std::endl;

  for (int i = 0; i < rvecsLeft.size(); i++)
  {
    std::ostringstream fileName;
    fileName << "/Users/mattclarkson/calib.before.left." << i << ".4x4";
    cv::Matx44d extrinsics = niftk::RodriguesToMatrix(rvecsLeft[i], tvecsLeft[i]);
    cv::Matx44d inverted = extrinsics.inv();
    std::ofstream f;
    f.open(fileName.str());
    f << inverted(0, 0) << " " << inverted(0, 1) << " " << inverted(0, 2) << " " << inverted(0, 3) << std::endl;
    f << inverted(1, 0) << " " << inverted(1, 1) << " " << inverted(1, 2) << " " << inverted(1, 3) << std::endl;
    f << inverted(2, 0) << " " << inverted(2, 1) << " " << inverted(2, 2) << " " << inverted(2, 3) << std::endl;
    f << inverted(3, 0) << " " << inverted(3, 1) << " " << inverted(3, 2) << " " << inverted(3, 3) << std::endl;
  }

  for (int i = 0; i < rvecsRight.size(); i++)
  {
    std::ostringstream fileName;
    fileName << "/Users/mattclarkson/calib.before.right." << i << ".4x4";
    cv::Matx44d extrinsics = niftk::RodriguesToMatrix(rvecsRight[i], tvecsRight[i]);
    cv::Matx44d inverted = extrinsics.inv();
    std::ofstream f;
    f.open(fileName.str());
    f << inverted(0, 0) << " " << inverted(0, 1) << " " << inverted(0, 2) << " " << inverted(0, 3) << std::endl;
    f << inverted(1, 0) << " " << inverted(1, 1) << " " << inverted(1, 2) << " " << inverted(1, 3) << std::endl;
    f << inverted(2, 0) << " " << inverted(2, 1) << " " << inverted(2, 2) << " " << inverted(2, 3) << std::endl;
    f << inverted(3, 0) << " " << inverted(3, 1) << " " << inverted(3, 2) << " " << inverted(3, 3) << std::endl;
  }

  std::cerr << "Matt, l2r=" << leftToRightTranslationVector << std::endl;

  // Ensure rhs extrinsics are set from lhs and left-to-right transform.
  niftk::ComputeStereoExtrinsics(rvecsLeft,
                                 tvecsLeft,
                                 leftToRightRotationMatrix,
                                 leftToRightTranslationVector,
                                 rvecsRight,
                                 tvecsRight
                                );

  // Recompute re-projection error, now we have updated the right extrinsics.
  projectedRMS = niftk::ComputeRMSReprojectionError(model,
                                                    listOfLeftHandPointSets,
                                                    listOfRightHandPointSets,
                                                    intrinsicLeft,
                                                    distortionLeft,
                                                    rvecsLeft,
                                                    tvecsLeft,
                                                    intrinsicRight,
                                                    distortionRight,
                                                    leftToRightRotationMatrix,
                                                    leftToRightTranslationVector
                                                   );

  // Compute 3D reconstruction error, now we have updated the right extrinsics.
  reconstructedRMS = niftk::ComputeRMSReconstructionError(model,
                                                          listOfLeftHandPointSets,
                                                          listOfRightHandPointSets,
                                                          intrinsicLeft,
                                                          distortionLeft,
                                                          rvecsLeft,
                                                          tvecsLeft,
                                                          intrinsicRight,
                                                          distortionRight,
                                                          leftToRightRotationMatrix,
                                                          leftToRightTranslationVector,
                                                          rmsInEachAxis
                                                         );

  std::cout << "niftkStereoCameraCalibration:Our optimisation with recomputed metrics, rms2D=" << projectedRMS
            << ", rms3D=" << reconstructedRMS << std::endl;

  double rmsTolerance = 0.0001;
  double previousRMS = projectedRMS + 2 * rmsTolerance;
  double currentRMS = projectedRMS;
/*
  while (fabs(currentRMS - previousRMS) >= rmsTolerance)
  {
    previousRMS = currentRMS;

    // Then optimise left camera extrinsics (camera positions), and optionally intrinsics.
    niftk::NonLinearStereoCameraCalibration2DOptimiser::Pointer optimiser2D
        = niftk::NonLinearStereoCameraCalibration2DOptimiser::New();
    optimiser2D->SetModelAndPoints(tmpModel, &listOfLeftHandPointSets, &listOfRightHandPointSets);
    optimiser2D->SetDistortion(&distortionLeft);
    optimiser2D->SetRightDistortion(&distortionRight);
    optimiser2D->SetIntrinsic(&intrinsicLeft);
    optimiser2D->SetRightIntrinsic(&intrinsicRight);
    optimiser2D->SetExtrinsics(rvecsLeft, tvecsLeft);

    if (cvFlags & CV_CALIB_FIX_INTRINSIC)
    {
      optimiser2D->SetOptimiseIntrinsics(false);
    }
    else
    {
      optimiser2D->SetOptimiseIntrinsics(true);
    }

    optimiser2D->SetOptimise2DOFStereo(false);  // not fully tested, so not in public API.
    optimiser2D->SetForceUnitVectorAxes(false); // not fully tested, so not in public API.
    optimiser2D->SetOptimiseExtrinsics(true);
    optimiser2D->SetOptimiseR2L(false);

    currentRMS = optimiser2D->Optimise(intrinsicLeft,
                                       intrinsicRight,
                                       rvecsLeft,
                                       tvecsLeft,
                                       leftToRightRotationMatrix,
                                       leftToRightTranslationVector
                                      );

    std::cout << "niftkStereoCameraCalibration::Optimised extrinsics rms2D = " << currentRMS << std::endl;

    // Then optimise stereo extrinsics (left-to-right), and optionally intrinsics.
    optimiser2D->SetOptimiseExtrinsics(false);
    optimiser2D->SetOptimiseR2L(true);

    currentRMS = optimiser2D->Optimise(intrinsicLeft,
                                       intrinsicRight,
                                       rvecsLeft,
                                       tvecsLeft,
                                       leftToRightRotationMatrix,
                                       leftToRightTranslationVector
                                      );

    std::cout << "niftkStereoCameraCalibration::Optimised r2l rms2D = " << currentRMS << std::endl;

  }
*/
#endif
/*
  // Ensure rhs extrinsics are set from lhs and left-to-right
  niftk::ComputeStereoExtrinsics(rvecsLeft,
                                 tvecsLeft,
                                 leftToRightRotationMatrix,
                                 leftToRightTranslationVector,
                                 rvecsRight,
                                 tvecsRight
                                );

  // Re-compute 3D reconstruction error
  reconstructedRMS = niftk::ComputeRMSReconstructionError(model,
                                                          listOfLeftHandPointSets,
                                                          listOfRightHandPointSets,
                                                          intrinsicLeft,
                                                          distortionLeft,
                                                          rvecsLeft,
                                                          tvecsLeft,
                                                          intrinsicRight,
                                                          distortionRight,
                                                          leftToRightRotationMatrix,
                                                          leftToRightTranslationVector,
                                                          rmsInEachAxis
                                                         );

  // Recompute re-projection error, now we have updated the left/right extrinsics.
  projectedRMS = niftk::ComputeRMSReprojectionError(model,
                                                    listOfLeftHandPointSets,
                                                    listOfRightHandPointSets,
                                                    intrinsicLeft,
                                                    distortionLeft,
                                                    rvecsLeft,
                                                    tvecsLeft,
                                                    intrinsicRight,
                                                    distortionRight,
                                                    leftToRightRotationMatrix,
                                                    leftToRightTranslationVector
                                                   );

  std::cout << "niftkStereoCameraCalibration:2D optimisation finished,     rms2D=" << projectedRMS
            << ", rms3D=" << reconstructedRMS << std::endl;
*/
#ifdef NIFTYCAL_WITH_ITK
  if (optimise3D)
  {
    rmsTolerance = 0.005;
    previousRMS = reconstructedRMS + 2 * rmsTolerance;
    currentRMS = reconstructedRMS;
    unsigned int numberOfIterations = 0;
    unsigned int maxNumberOfIterations = 20;
    while (fabs(currentRMS - previousRMS) >= rmsTolerance
           && numberOfIterations < maxNumberOfIterations
          )
    {
      previousRMS = currentRMS;

      // Now optimise RMS reconstruction error via extrinsics.
      niftk::NonLinearStereoExtrinsicsCalibration3DOptimiser::Pointer extrinsicsOptimiser =
          niftk::NonLinearStereoExtrinsicsCalibration3DOptimiser::New();

      extrinsicsOptimiser->SetModelAndPoints(tmpModel,
                                             &listOfLeftHandPointSets,
                                             &listOfRightHandPointSets);

      extrinsicsOptimiser->SetIntrinsics(&intrinsicLeft,
                                         &intrinsicRight
                                        );

      extrinsicsOptimiser->SetDistortionParameters(&distortionLeft, &distortionRight);
      extrinsicsOptimiser->SetOptimiseCameraExtrinsics(true);
      extrinsicsOptimiser->SetOptimiseL2R(false);

      currentRMS = extrinsicsOptimiser->Optimise(rvecsLeft,
                                                 tvecsLeft,
                                                 leftToRightRotationMatrix,
                                                 leftToRightTranslationVector
                                                );

      std::cout << "niftkStereoCameraCalibration::Optimised extrinsics rms3D = " << currentRMS << std::endl;

      extrinsicsOptimiser->SetOptimiseCameraExtrinsics(false);
      extrinsicsOptimiser->SetOptimiseL2R(true);

      currentRMS = extrinsicsOptimiser->Optimise(rvecsLeft,
                                                 tvecsLeft,
                                                 leftToRightRotationMatrix,
                                                 leftToRightTranslationVector
                                                );

      std::cout << "niftkStereoCameraCalibration::Optimised r2l rms3D = " << currentRMS << std::endl;

      numberOfIterations++;
    }

    reconstructedRMS = currentRMS;

    // Ensure rhs extrinsics are set from lhs and left-to-right
    niftk::ComputeStereoExtrinsics(rvecsLeft,
                                   tvecsLeft,
                                   leftToRightRotationMatrix,
                                   leftToRightTranslationVector,
                                   rvecsRight,
                                   tvecsRight
                                  );

    // Recompute re-projection error, as it will now be different.
    projectedRMS = niftk::ComputeRMSReprojectionError(model,
                                                      listOfLeftHandPointSets,
                                                      listOfRightHandPointSets,
                                                      intrinsicLeft,
                                                      distortionLeft,
                                                      rvecsLeft,
                                                      tvecsLeft,
                                                      intrinsicRight,
                                                      distortionRight,
                                                      leftToRightRotationMatrix,
                                                      leftToRightTranslationVector
                                                     );

    std::cout << "niftkStereoCameraCalibration:3D optimisation finished,     rms2D=" << projectedRMS
              << ", rms3D=" << reconstructedRMS << std::endl;
  }

#endif

  result(0, 0) = projectedRMS;
  result(1, 0) = reconstructedRMS;

  return result;
}


//-----------------------------------------------------------------------------
cv::Matx21d FullStereoCameraCalibration(const Model3D& model,
                                        const std::list<PointSet>& leftPoints,
                                        const std::list<PointSet>& rightPoints,
                                        const cv::Size2i& imageSize,
                                        cv::Mat& intrinsicLeft,
                                        cv::Mat& distortionLeft,
                                        std::vector<cv::Mat>& rvecsLeft,
                                        std::vector<cv::Mat>& tvecsLeft,
                                        cv::Mat& intrinsicRight,
                                        cv::Mat& distortionRight,
                                        std::vector<cv::Mat>& rvecsRight,
                                        std::vector<cv::Mat>& tvecsRight,
                                        cv::Mat& leftToRightRotationMatrix,
                                        cv::Mat& leftToRightTranslationVector,
                                        cv::Mat& essentialMatrix,
                                        cv::Mat& fundamentalMatrix,
                                        const bool& optimise3D
                                       )
{
  cv::Matx21d result;

  // For niftkStereoSimulationWithNoise, we may pass a list of two in here,
  // but only want to use the first one for Tsai calibration.
  if (leftPoints.size() < 3 && rightPoints.size() < 3)
  {
    cv::Mat rvecLeft;
    cv::Mat tvecLeft;
    cv::Mat rvecRight;
    cv::Mat tvecRight;

    cv::Point2d sensorDimensions;
    sensorDimensions.x = 1;
    sensorDimensions.y = 1;

    niftk::TsaiMonoCameraCalibration(model,
                                     *(leftPoints.begin()),
                                     imageSize,
                                     sensorDimensions,
                                     intrinsicLeft,
                                     distortionLeft,
                                     rvecLeft,
                                     tvecLeft,
                                     true // full optimisation.
                                    );

    niftk::TsaiMonoCameraCalibration(model,
                                     *(rightPoints.begin()),
                                     imageSize,
                                     sensorDimensions,
                                     intrinsicRight,
                                     distortionRight,
                                     rvecRight,
                                     tvecRight,
                                     true // full optimisation.
                                    );

    result = niftk::TsaiStereoCameraCalibration(model,
                                                *(leftPoints.begin()),
                                                *(rightPoints.begin()),
                                                imageSize,
                                                intrinsicLeft,
                                                distortionLeft,
                                                rvecLeft,
                                                tvecLeft,
                                                intrinsicRight,
                                                distortionRight,
                                                rvecRight,
                                                tvecRight,
                                                leftToRightRotationMatrix,
                                                leftToRightTranslationVector,
                                                essentialMatrix,
                                                fundamentalMatrix,
                                                CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_INTRINSIC,
                                                optimise3D
                                               );

    rvecsLeft.clear();
    tvecsLeft.clear();
    rvecsRight.clear();
    tvecsRight.clear();

    rvecsLeft.push_back(rvecLeft);
    tvecsLeft.push_back(tvecLeft);
    rvecsRight.push_back(rvecRight);
    tvecsRight.push_back(tvecRight);
  }
  else
  {
    niftk::ZhangMonoCameraCalibration(model,
                                      leftPoints,
                                      imageSize,
                                      intrinsicLeft,
                                      distortionLeft,
                                      rvecsLeft,
                                      tvecsLeft,
                                      CV_CALIB_FIX_PRINCIPAL_POINT
                                      | CV_CALIB_FIX_K2
                                      | CV_CALIB_FIX_K3
                                      | CV_CALIB_FIX_K4
                                      | CV_CALIB_FIX_K5
                                      | CV_CALIB_FIX_K6
                                     );

    niftk::ZhangMonoCameraCalibration(model,
                                      rightPoints,
                                      imageSize,
                                      intrinsicRight,
                                      distortionRight,
                                      rvecsRight,
                                      tvecsRight,
                                      CV_CALIB_FIX_PRINCIPAL_POINT
                                      | CV_CALIB_FIX_K2
                                      | CV_CALIB_FIX_K3
                                      | CV_CALIB_FIX_K4
                                      | CV_CALIB_FIX_K5
                                      | CV_CALIB_FIX_K6
                                     );

    result = niftk::StereoCameraCalibration(model,
                                            leftPoints,
                                            rightPoints,
                                            imageSize,
                                            intrinsicLeft,
                                            distortionLeft,
                                            rvecsLeft,
                                            tvecsLeft,
                                            intrinsicRight,
                                            distortionRight,
                                            rvecsRight,
                                            tvecsRight,
                                            leftToRightRotationMatrix,
                                            leftToRightTranslationVector,
                                            essentialMatrix,
                                            fundamentalMatrix,
                                            CV_CALIB_USE_INTRINSIC_GUESS
                                            | CV_CALIB_FIX_INTRINSIC
                                            ,
                                            optimise3D
                                           );
  }
  return result;
}

} // end namespace
