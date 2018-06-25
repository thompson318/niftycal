/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearCeresMonoOptimiser.h"
#include <niftkMatrixUtilities.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <iostream>
#include <vector>

namespace niftk
{

/**
 * \brief Internal function to compute the re-projection point on one camera.
 */
template <typename T>
void ProjectToPoint(const T* const intrinsic,
                    const T* const distortion,
                    const T* const cameraPoint,
                    T& x,
                    T& y
                   )
{
  // Normalise
  T xp = cameraPoint[0] / cameraPoint[2];
  T yp = cameraPoint[1] / cameraPoint[2];

  // Apply distortion
  // distortion[0] = k1
  // distortion[1] = k2
  // distortion[2] = p1
  // distortion[3] = p2
  // distortion[4] = k3
  T r2 = xp*xp + yp*yp;
  T xpd = xp * (T(1) + distortion[0]*r2 + distortion[1]*r2*r2 + distortion[4]*r2*r2*r2)
          + T(2)*distortion[2]*xp*yp + distortion[3]*(r2 + T(2)*xp*xp);
  T ypd = yp * (T(1) + distortion[0]*r2 + distortion[1]*r2*r2 + distortion[4]*r2*r2*r2)
          + T(2)*distortion[3]*xp*yp + distortion[2]*(r2 + T(2)*yp*yp);

  x = intrinsic[0]*xpd + intrinsic[2];
  y = intrinsic[1]*ypd + intrinsic[3];
}

struct MonoCumulativeProjectionConstraint {
public:
  MonoCumulativeProjectionConstraint(const std::vector<std::vector<cv::Vec3f> >& m,
                                     const std::vector<std::vector<cv::Vec2f> >& i
                                    )
  : m_ModelPoints(m)
  , m_ImagePoints(i)
  {
  }

  /**
   * Lets define
   * intrinsic (4) (0-3)
   * distortion (5) (4-8)
   * initial transform (9-14)
   * Extrinsic parameters for each camera (15 onwards)
   */
  template <typename T>
  bool operator()(T const* const* parameters,
                  T* residuals) const {

    unsigned long long int residualCounter = 0;
    for (std::vector<std::vector<cv::Vec3f> >::size_type i = 0; i < m_ModelPoints.size(); i++)
    {
      for (std::vector<cv::Point3f>::size_type p = 0; p < m_ModelPoints[i].size(); p++)
      {
        T m[3];
        m[0] = T(static_cast<double>(m_ModelPoints[i][p](0)));
        m[1] = T(static_cast<double>(m_ModelPoints[i][p](1)));
        m[2] = T(static_cast<double>(m_ModelPoints[i][p](2)));

        // Initial transform
        T l[3];
        ceres::AngleAxisRotatePoint(parameters[2], m, l);
        l[0] += (parameters[2])[3];
        l[1] += (parameters[2])[4];
        l[2] += (parameters[2])[5];

        // Now accumulate all the intra-camera offsets.
        for (int j = 0; j <= i; j++)
        {
          T tmp[3];
          tmp[0] = l[0];
          tmp[1] = l[1];
          tmp[2] = l[2];
          ceres::AngleAxisRotatePoint(&((parameters[3])[j*6 + 0]), tmp, l);
          l[0] += (parameters[3])[j*6 + 3];
          l[1] += (parameters[3])[j*6 + 4];
          l[2] += (parameters[3])[j*6 + 5];
        }

        T lx;
        T ly;
        ProjectToPoint<T>(parameters[0],
                          parameters[1],
                          l,
                          lx,
                          ly
                         );

        residuals[residualCounter++] = lx - T(static_cast<double>(m_ImagePoints[i][p](0)));
        residuals[residualCounter++] = ly - T(static_cast<double>(m_ImagePoints[i][p](1)));
      }
    }
    return true;
  }

  std::vector<std::vector<cv::Vec3f> >  m_ModelPoints;
  std::vector<std::vector<cv::Vec2f> >  m_ImagePoints;
};


//-----------------------------------------------------------------------------
double CeresMonoCameraCalibration(const std::vector<std::vector<cv::Vec3f> >& modelVectors3D,
                                  const std::vector<std::vector<cv::Vec2f> >& imageVectors2D,
                                  cv::Mat& intrinsic,
                                  cv::Mat& distortion,
                                  std::vector<cv::Mat>& rvecs,
                                  std::vector<cv::Mat>& tvecs
                                 )
{
  unsigned long long int numberOfPoints = 0;
  for (std::vector<std::vector<cv::Vec3f> >::size_type i = 0; i < modelVectors3D.size(); i++)
  {
    numberOfPoints += modelVectors3D[i].size();
  }

  const unsigned int numberOfParameters = 4 // intrinsic
                                        + 5 // distortion (not optimised)
                                        + 6 // first left transformation (not optimised)
                                        + 6*rvecs.size(); // extrinsics for each left-hand camera

  double *parameters = new double[numberOfParameters];

  unsigned int parameterCounter = 0;
  parameters[parameterCounter++] = intrinsic.at<double>(0, 0);
  parameters[parameterCounter++] = intrinsic.at<double>(1, 1);
  parameters[parameterCounter++] = intrinsic.at<double>(0, 2);
  parameters[parameterCounter++] = intrinsic.at<double>(1, 2);
  parameters[parameterCounter++] = distortion.at<double>(0, 0);
  parameters[parameterCounter++] = distortion.at<double>(0, 1);
  parameters[parameterCounter++] = distortion.at<double>(0, 2);
  parameters[parameterCounter++] = distortion.at<double>(0, 3);
  parameters[parameterCounter++] = distortion.at<double>(0, 4);

  parameters[parameterCounter++] = rvecs[0].at<double>(0, 0);
  parameters[parameterCounter++] = rvecs[0].at<double>(0, 1);
  parameters[parameterCounter++] = rvecs[0].at<double>(0, 2);
  parameters[parameterCounter++] = tvecs[0].at<double>(0, 0);
  parameters[parameterCounter++] = tvecs[0].at<double>(0, 1);
  parameters[parameterCounter++] = tvecs[0].at<double>(0, 2);

  // So the parameters for the first camera that are optimised, start at the identity transform.
  cv::Matx44d identity = cv::Matx44d::eye();
  cv::Mat firstCameraRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat firstCameraTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  niftk::MatrixToRodrigues(identity, firstCameraRotationVector, firstCameraTranslationVector);
  parameters[parameterCounter++] = firstCameraRotationVector.at<double>(0, 0);
  parameters[parameterCounter++] = firstCameraRotationVector.at<double>(0, 1);
  parameters[parameterCounter++] = firstCameraRotationVector.at<double>(0, 2);
  parameters[parameterCounter++] = firstCameraTranslationVector.at<double>(0, 0);
  parameters[parameterCounter++] = firstCameraTranslationVector.at<double>(0, 1);
  parameters[parameterCounter++] = firstCameraTranslationVector.at<double>(0, 2);

  // Then for all other cameras, we go relative to the previous camera.
  for (int i = 1; i < rvecs.size(); i++)
  {
    cv::Matx44d previousCamera = niftk::RodriguesToMatrix(rvecs[i-1], tvecs[i-1]);
    cv::Matx44d currentCamera = niftk::RodriguesToMatrix(rvecs[i], tvecs[i]);
    cv::Matx44d previousCameraToCurrentCamera = currentCamera * previousCamera.inv();

    cv::Mat previousCameraToCurrentCameraRotationVector = cvCreateMat(1, 3, CV_64FC1);
    cv::Mat previousCameraToCurrentCameraTranslationVector = cvCreateMat(1, 3, CV_64FC1);
    niftk::MatrixToRodrigues(previousCameraToCurrentCamera,
                             previousCameraToCurrentCameraRotationVector,
                             previousCameraToCurrentCameraTranslationVector);

    parameters[parameterCounter++] = previousCameraToCurrentCameraRotationVector.at<double>(0, 0);
    parameters[parameterCounter++] = previousCameraToCurrentCameraRotationVector.at<double>(0, 1);
    parameters[parameterCounter++] = previousCameraToCurrentCameraRotationVector.at<double>(0, 2);
    parameters[parameterCounter++] = previousCameraToCurrentCameraTranslationVector.at<double>(0, 0);
    parameters[parameterCounter++] = previousCameraToCurrentCameraTranslationVector.at<double>(0, 1);
    parameters[parameterCounter++] = previousCameraToCurrentCameraTranslationVector.at<double>(0, 2);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Problem problem;
  ceres::DynamicAutoDiffCostFunction<MonoCumulativeProjectionConstraint> *costFunction =
    new ceres::DynamicAutoDiffCostFunction<MonoCumulativeProjectionConstraint>(
      new MonoCumulativeProjectionConstraint(modelVectors3D, imageVectors2D)
      );

  std::vector<double*> parameterBlocks;
  parameterBlocks.push_back(&parameters[0]);
  costFunction->AddParameterBlock(4);
  parameterBlocks.push_back(&parameters[4]);
  costFunction->AddParameterBlock(5);
  parameterBlocks.push_back(&parameters[9]);
  costFunction->AddParameterBlock(6);
  parameterBlocks.push_back(&parameters[15]);
  costFunction->AddParameterBlock(6 * modelVectors3D.size());
  costFunction->SetNumResiduals(2 * numberOfPoints);

  problem.AddResidualBlock(costFunction,
                           NULL,
                           parameterBlocks
                          );

  problem.SetParameterBlockConstant(&parameters[4]); // distortion
  problem.SetParameterBlockConstant(&parameters[9]); // fixed transform

  // Run the solver!
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  // Unpack the parameters
  parameterCounter = 0;
  intrinsic.at<double>(0, 0) = parameters[parameterCounter++];
  intrinsic.at<double>(1, 1) = parameters[parameterCounter++];
  intrinsic.at<double>(0, 2) = parameters[parameterCounter++];
  intrinsic.at<double>(1, 2) = parameters[parameterCounter++];
  distortion.at<double>(0, 0) = parameters[parameterCounter++];
  distortion.at<double>(0, 1) = parameters[parameterCounter++];
  distortion.at<double>(0, 2) = parameters[parameterCounter++];
  distortion.at<double>(0, 3) = parameters[parameterCounter++];
  distortion.at<double>(0, 4) = parameters[parameterCounter++];

  firstCameraRotationVector.at<double>(0, 0) = parameters[parameterCounter++];
  firstCameraRotationVector.at<double>(0, 1) = parameters[parameterCounter++];
  firstCameraRotationVector.at<double>(0, 2) = parameters[parameterCounter++];
  firstCameraTranslationVector.at<double>(0, 0) = parameters[parameterCounter++];
  firstCameraTranslationVector.at<double>(0, 1) = parameters[parameterCounter++];
  firstCameraTranslationVector.at<double>(0, 2) = parameters[parameterCounter++];
  cv::Matx44d firstCamera = niftk::RodriguesToMatrix(firstCameraRotationVector, firstCameraTranslationVector);

  unsigned int cameraLimit = 0;
  for (unsigned int i = 0; i < rvecs.size(); i++)
  {
    cv::Matx44d cumulativeTransform = firstCamera;
    cv::Mat currentCameraRotationVector = cvCreateMat(1, 3, CV_64FC1);
    cv::Mat currentCameraTranslationVector = cvCreateMat(1, 3, CV_64FC1);

    unsigned int parameterCounterInsideLoop = parameterCounter;

    for (unsigned int j = 0; j <= cameraLimit && cameraLimit < rvecs.size(); j++)
    {
      currentCameraRotationVector.at<double>(0, 0) = parameters[parameterCounterInsideLoop++];
      currentCameraRotationVector.at<double>(0, 1) = parameters[parameterCounterInsideLoop++];
      currentCameraRotationVector.at<double>(0, 2) = parameters[parameterCounterInsideLoop++];

      currentCameraTranslationVector.at<double>(0, 0) = parameters[parameterCounterInsideLoop++];
      currentCameraTranslationVector.at<double>(0, 1) = parameters[parameterCounterInsideLoop++];
      currentCameraTranslationVector.at<double>(0, 2) = parameters[parameterCounterInsideLoop++];

      cv::Matx44d offset = niftk::RodriguesToMatrix(currentCameraRotationVector, currentCameraTranslationVector);
      cumulativeTransform = offset * cumulativeTransform;
    }

    niftk::MatrixToRodrigues(cumulativeTransform, currentCameraRotationVector, currentCameraTranslationVector);

    rvecs[i].at<double>(0, 0) = currentCameraRotationVector.at<double>(0, 0);
    rvecs[i].at<double>(0, 1) = currentCameraRotationVector.at<double>(0, 1);
    rvecs[i].at<double>(0, 2) = currentCameraRotationVector.at<double>(0, 2);
    tvecs[i].at<double>(0, 0) = currentCameraTranslationVector.at<double>(0, 0);
    tvecs[i].at<double>(0, 1) = currentCameraTranslationVector.at<double>(0, 1);
    tvecs[i].at<double>(0, 2) = currentCameraTranslationVector.at<double>(0, 2);

    cameraLimit++;
  }

  delete [] parameters;

  return summary.final_cost;
}

} // end namespace
