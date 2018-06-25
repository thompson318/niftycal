/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearCeresStereoOptimiser.h"
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

struct LeftProjectionConstraint {
public:
  LeftProjectionConstraint(const std::vector<cv::Vec3f>& m,
                           const std::vector<cv::Vec2f>& l
                          )
  : m_ModelPoints(m)
  , m_ImagePoints(l)
  {
  }

  template <typename T>
  bool operator()(T const* const* parameters,
                  T* residuals) const {

    unsigned long long int residualCounter = 0;
    for (std::vector<cv::Point3f>::size_type p = 0; p < m_ModelPoints.size(); p++)
    {
      T m[3];
      m[0] = T(static_cast<double>(m_ModelPoints[p](0)));
      m[1] = T(static_cast<double>(m_ModelPoints[p](1)));
      m[2] = T(static_cast<double>(m_ModelPoints[p](2)));

      T l[3];
      ceres::AngleAxisRotatePoint(parameters[2], m, l);
      l[0] += (parameters[2])[3];
      l[1] += (parameters[2])[4];
      l[2] += (parameters[2])[5];

      T lx;
      T ly;
      ProjectToPoint<T>(parameters[0],
                        parameters[1],
                        l,
                        lx,
                        ly
                       );

      residuals[residualCounter++] = lx - T(static_cast<double>(m_ImagePoints[p](0)));
      residuals[residualCounter++] = ly - T(static_cast<double>(m_ImagePoints[p](1)));
    }
    return true;
  }

  std::vector<cv::Vec3f> m_ModelPoints;
  std::vector<cv::Vec2f> m_ImagePoints;
};


struct RightProjectionConstraint {
public:
  RightProjectionConstraint(const std::vector<cv::Vec3f>& m,
                            const std::vector<cv::Vec2f>& r
                           )
  : m_ModelPoints(m)
  , m_ImagePoints(r)
  {
  }

  template <typename T>
  bool operator()(T const* const* parameters,
                  T* residuals) const {

    unsigned long long int residualCounter = 0;
    for (std::vector<cv::Point3f>::size_type p = 0; p < m_ModelPoints.size(); p++)
    {
      T m[3];
      m[0] = T(static_cast<double>(m_ModelPoints[p](0)));
      m[1] = T(static_cast<double>(m_ModelPoints[p](1)));
      m[2] = T(static_cast<double>(m_ModelPoints[p](2)));

      T l[3];
      ceres::AngleAxisRotatePoint(&((parameters[3])[0]), m, l);
      l[0] += (parameters[3])[3];
      l[1] += (parameters[3])[4];
      l[2] += (parameters[3])[5];

      T r[3];
      ceres::AngleAxisRotatePoint(parameters[2], l, r);
      r[0] += (parameters[2])[3];
      r[1] += (parameters[2])[4];
      r[2] += (parameters[2])[5];

      T rx;
      T ry;
      ProjectToPoint<T>(parameters[0],
                        parameters[1],
                        r,
                        rx,
                        ry
                       );

      residuals[residualCounter++] = rx - T(static_cast<double>(m_ImagePoints[p](0)));
      residuals[residualCounter++] = ry - T(static_cast<double>(m_ImagePoints[p](1)));
    }
    return true;
  }

  std::vector<cv::Vec3f> m_ModelPoints;
  std::vector<cv::Vec2f> m_ImagePoints;
};


//-----------------------------------------------------------------------------
double CeresStereoCameraCalibration(const std::vector<std::vector<cv::Vec3f> >& objectVectors3D,
                                  const std::vector<std::vector<cv::Vec2f> >& leftVectors2D,
                                  const std::vector<std::vector<cv::Vec2f> >& rightVectors2D,
                                  cv::Mat& intrinsicLeft,
                                  cv::Mat& distortionLeft,
                                  std::vector<cv::Mat>& rvecsLeft,
                                  std::vector<cv::Mat>& tvecsLeft,
                                  cv::Mat& intrinsicRight,
                                  cv::Mat& distortionRight,
                                  cv::Mat& leftToRightRotationMatrix,
                                  cv::Mat& leftToRightTranslationVector
                                 )
{
  unsigned long long int numberOfPoints = 0;
  for (std::vector<std::vector<cv::Vec3f> >::size_type i = 0; i < objectVectors3D.size(); i++)
  {
    numberOfPoints += objectVectors3D[i].size();
  }

  const unsigned int numberOfParameters = 4 // intrinsic left
                                        + 5 // distortion left
                                        + 4 // intrinsic right
                                        + 5 // distortion right
                                        + 6 // left to right transform
                                        + 6*rvecsLeft.size(); // extrinsics for each left-hand camera

  double *parameters = new double[numberOfParameters];

  unsigned int parameterCounter = 0;
  parameters[parameterCounter++] = intrinsicLeft.at<double>(0, 0);
  parameters[parameterCounter++] = intrinsicLeft.at<double>(1, 1);
  parameters[parameterCounter++] = intrinsicLeft.at<double>(0, 2);
  parameters[parameterCounter++] = intrinsicLeft.at<double>(1, 2);
  parameters[parameterCounter++] = distortionLeft.at<double>(0, 0);
  parameters[parameterCounter++] = distortionLeft.at<double>(0, 1);
  parameters[parameterCounter++] = distortionLeft.at<double>(0, 2);
  parameters[parameterCounter++] = distortionLeft.at<double>(0, 3);
  parameters[parameterCounter++] = distortionLeft.at<double>(0, 4);
  parameters[parameterCounter++] = intrinsicRight.at<double>(0, 0);
  parameters[parameterCounter++] = intrinsicRight.at<double>(1, 1);
  parameters[parameterCounter++] = intrinsicRight.at<double>(0, 2);
  parameters[parameterCounter++] = intrinsicRight.at<double>(1, 2);
  parameters[parameterCounter++] = distortionRight.at<double>(0, 0);
  parameters[parameterCounter++] = distortionRight.at<double>(0, 1);
  parameters[parameterCounter++] = distortionRight.at<double>(0, 2);
  parameters[parameterCounter++] = distortionRight.at<double>(0, 3);
  parameters[parameterCounter++] = distortionRight.at<double>(0, 4);

  cv::Mat leftToRightRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);

  parameters[parameterCounter++] = leftToRightRotationVector.at<double>(0, 0);
  parameters[parameterCounter++] = leftToRightRotationVector.at<double>(0, 1);
  parameters[parameterCounter++] = leftToRightRotationVector.at<double>(0, 2);
  parameters[parameterCounter++] = leftToRightTranslationVector.at<double>(0, 0);
  parameters[parameterCounter++] = leftToRightTranslationVector.at<double>(0, 1);
  parameters[parameterCounter++] = leftToRightTranslationVector.at<double>(0, 2);

  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    parameters[parameterCounter++] = rvecsLeft[i].at<double>(0, 0);
    parameters[parameterCounter++] = rvecsLeft[i].at<double>(0, 1);
    parameters[parameterCounter++] = rvecsLeft[i].at<double>(0, 2);
    parameters[parameterCounter++] = tvecsLeft[i].at<double>(0, 0);
    parameters[parameterCounter++] = tvecsLeft[i].at<double>(0, 1);
    parameters[parameterCounter++] = tvecsLeft[i].at<double>(0, 2);
  }

  double *initialParameters = new double[numberOfParameters];
  for (unsigned int i = 0; i < numberOfParameters; i++)
  {
    initialParameters[i] = parameters[i];
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Problem problem;

  // Cost functions for each left hand camera.
  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    ceres::DynamicAutoDiffCostFunction<LeftProjectionConstraint> *leftCostFunction =
      new ceres::DynamicAutoDiffCostFunction<LeftProjectionConstraint>(
        new LeftProjectionConstraint(objectVectors3D[i], leftVectors2D[i])
        );
    std::vector<double*> leftBlocks;
    leftBlocks.push_back(&parameters[0]);
    leftCostFunction->AddParameterBlock(4);
    leftBlocks.push_back(&parameters[4]);
    leftCostFunction->AddParameterBlock(5);
    leftBlocks.push_back(&parameters[24 + i*6]);
    leftCostFunction->AddParameterBlock(6);
    leftCostFunction->SetNumResiduals(2 * objectVectors3D[i].size());

    problem.AddResidualBlock(leftCostFunction,
                             NULL,
                             leftBlocks
                            );

    problem.SetParameterBlockConstant(&parameters[0]);
    problem.SetParameterBlockConstant(&parameters[4]);

    ceres::DynamicAutoDiffCostFunction<RightProjectionConstraint> *rightCostFunction =
      new ceres::DynamicAutoDiffCostFunction<RightProjectionConstraint>(
        new RightProjectionConstraint(objectVectors3D[i], rightVectors2D[i])
        );

    std::vector<double*> rightBlocks;
    rightBlocks.push_back(&parameters[9]);
    rightCostFunction->AddParameterBlock(4);
    rightBlocks.push_back(&parameters[13]);
    rightCostFunction->AddParameterBlock(5);
    rightBlocks.push_back(&parameters[18]);
    rightCostFunction->AddParameterBlock(6);
    rightBlocks.push_back(&parameters[24 + i*6]);
    rightCostFunction->AddParameterBlock(6);
    rightCostFunction->SetNumResiduals(2 * objectVectors3D[i].size());

    problem.AddResidualBlock(rightCostFunction,
                             NULL,
                             rightBlocks
                            );

    problem.SetParameterBlockConstant(&parameters[9]);
    problem.SetParameterBlockConstant(&parameters[13]);
  }

  // Run the solver!
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  parameterCounter = 0;
  intrinsicLeft.at<double>(0, 0) = parameters[parameterCounter++];
  intrinsicLeft.at<double>(1, 1) = parameters[parameterCounter++];
  intrinsicLeft.at<double>(0, 2) = parameters[parameterCounter++];
  intrinsicLeft.at<double>(1, 2) = parameters[parameterCounter++];
  distortionLeft.at<double>(0, 0) = parameters[parameterCounter++];
  distortionLeft.at<double>(0, 1) = parameters[parameterCounter++];
  distortionLeft.at<double>(0, 2) = parameters[parameterCounter++];
  distortionLeft.at<double>(0, 3) = parameters[parameterCounter++];
  distortionLeft.at<double>(0, 4) = parameters[parameterCounter++];
  intrinsicRight.at<double>(0, 0) = parameters[parameterCounter++];
  intrinsicRight.at<double>(1, 1) = parameters[parameterCounter++];
  intrinsicRight.at<double>(0, 2) = parameters[parameterCounter++];
  intrinsicRight.at<double>(1, 2) = parameters[parameterCounter++];
  distortionRight.at<double>(0, 0) = parameters[parameterCounter++];
  distortionRight.at<double>(0, 1) = parameters[parameterCounter++];
  distortionRight.at<double>(0, 2) = parameters[parameterCounter++];
  distortionRight.at<double>(0, 3) = parameters[parameterCounter++];
  distortionRight.at<double>(0, 4) = parameters[parameterCounter++];

  leftToRightRotationVector.at<double>(0, 0) = parameters[parameterCounter++];
  leftToRightRotationVector.at<double>(0, 1) = parameters[parameterCounter++];
  leftToRightRotationVector.at<double>(0, 2) = parameters[parameterCounter++];

  cv::Rodrigues(leftToRightRotationVector, leftToRightRotationMatrix);

  leftToRightTranslationVector.at<double>(0, 0) = parameters[parameterCounter++];
  leftToRightTranslationVector.at<double>(0, 1) = parameters[parameterCounter++];
  leftToRightTranslationVector.at<double>(0, 2) = parameters[parameterCounter++];

  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    rvecsLeft[i].at<double>(0, 0) = parameters[parameterCounter++];
    rvecsLeft[i].at<double>(0, 1) = parameters[parameterCounter++];
    rvecsLeft[i].at<double>(0, 2) = parameters[parameterCounter++];
    tvecsLeft[i].at<double>(0, 0) = parameters[parameterCounter++];
    tvecsLeft[i].at<double>(0, 1) = parameters[parameterCounter++];
    tvecsLeft[i].at<double>(0, 2) = parameters[parameterCounter++];
  }

  for (unsigned int i = 0; i < numberOfParameters; i++)
  {
    std::cerr << "Matt, i=" << i << ", " << initialParameters[i] << ", " << parameters[i] << ", " << parameters[i] - initialParameters[i] << std::endl;
  }
  delete [] parameters;

  return summary.final_cost;
}

} // end namespace
