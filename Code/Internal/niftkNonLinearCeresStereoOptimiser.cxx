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


/**
 * \brief Internal function to compute the reconstructed point from triangulating between 2 cameras.
 */
template <typename T>
void ReconstructPoint(const T* const intrinsicLeft,
                      const T* const distortionLeft,
                      const T* const cameraPointLeft,
                      const T* const intrinsicRight,
                      const T* const distortionRight,
                      const T* const cameraPointRight,
                      T& x,
                      T& y,
                      T& z
                   )
{

}


struct StereoProjectionConstraint {
public:
  StereoProjectionConstraint(const unsigned int& cameraCounter,
                             const cv::Point3f& p,
                             const cv::Vec2f& l,
                             const cv::Vec2f& r
                            )
  : m_CameraCounter(cameraCounter)
  , m_ModelPoint(p)
  , m_ObservedL(l)
  , m_ObservedR(r)
  {
  }

  /**
   * Lets define
   * Left intrinsic (4) (0-3)
   * Left distortion (5) (4-8)
   * Right intrinsic (4) (9-12)
   * Right distortion (5) (13-17)
   * Left-to-right rotation and translation vector (6) (18-23)
   * Left camera rotation and translation vector (6) (24 onwards)
   */
  template <typename T>
  bool operator()(T const* const* parameters,
                  T* residuals) const {

    T m[3];
    m[0] = T(static_cast<double>(m_ModelPoint.x));
    m[1] = T(static_cast<double>(m_ModelPoint.y));
    m[2] = T(static_cast<double>(m_ModelPoint.z));

    T l[3];
    ceres::AngleAxisRotatePoint(parameters[6 + m_CameraCounter*2 + 0], m, l);
    l[0] += *(parameters[6 + m_CameraCounter*2 + 1] + 0);
    l[1] += *(parameters[6 + m_CameraCounter*2 + 1] + 1);
    l[2] += *(parameters[6 + m_CameraCounter*2 + 1] + 2);

    T lx;
    T ly;
    ProjectToPoint<T>(parameters[0],
                      parameters[1],
                      l,
                      lx,
                      ly
                     );

    T r[3];
    ceres::AngleAxisRotatePoint(parameters[4], l, r);
    r[0] += *(parameters[5] + 0);
    r[1] += *(parameters[5] + 1);
    r[2] += *(parameters[5] + 2);

    T rx;
    T ry;
    ProjectToPoint<T>(parameters[2],
                      parameters[3],
                      r,
                      rx,
                      ry
                     );

    residuals[0] = lx - T(static_cast<double>(m_ObservedL(0)));
    residuals[1] = ly - T(static_cast<double>(m_ObservedL(1)));
    residuals[2] = rx - T(static_cast<double>(m_ObservedR(0)));
    residuals[3] = ry - T(static_cast<double>(m_ObservedR(1)));

    return true;
  }

  static ceres::CostFunction* Create(unsigned int& cameraCounter,
                                     const cv::Vec3f& modelPoint,
                                     const cv::Vec2f& leftObservedPoint,
                                     const cv::Vec2f& rightObservedPoint) {
    return (new ceres::DynamicAutoDiffCostFunction<StereoProjectionConstraint>(
              new StereoProjectionConstraint(cameraCounter,
                                             modelPoint,
                                             leftObservedPoint,
                                             rightObservedPoint)));
  }

  unsigned int m_CameraCounter;
  cv::Point3f m_ModelPoint;
  cv::Vec2f   m_ObservedL;
  cv::Vec2f   m_ObservedR;
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

  const unsigned int numberOfParameters = 4 + 5 + 4 + 5 + 6 + 6*rvecsLeft.size();
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

  ceres::Problem problem;
  for (unsigned int cameraCounter = 0; cameraCounter < objectVectors3D.size(); cameraCounter++)
  {
    for (unsigned int pointCounter = 0; pointCounter < objectVectors3D[cameraCounter].size(); pointCounter++)
    {
      ceres::DynamicAutoDiffCostFunction<StereoProjectionConstraint> *costFunction =
        new ceres::DynamicAutoDiffCostFunction<StereoProjectionConstraint>(
          new StereoProjectionConstraint(cameraCounter,
                                         objectVectors3D[cameraCounter][pointCounter],
                                         leftVectors2D[cameraCounter][pointCounter],
                                         rightVectors2D[cameraCounter][pointCounter]));
      std::vector<double*> parameterBlocks;
      parameterBlocks.push_back(&parameters[0]);
      costFunction->AddParameterBlock(4);
      parameterBlocks.push_back(&parameters[4]);
      costFunction->AddParameterBlock(5);
      parameterBlocks.push_back(&parameters[9]);
      costFunction->AddParameterBlock(4);
      parameterBlocks.push_back(&parameters[13]);
      costFunction->AddParameterBlock(5);
      parameterBlocks.push_back(&parameters[18]);
      costFunction->AddParameterBlock(3);
      parameterBlocks.push_back(&parameters[21]);
      costFunction->AddParameterBlock(3);

      for (int i = 0; i < objectVectors3D.size(); i++)
      {
        parameterBlocks.push_back(&parameters[24 + i*6 + 0]);
        costFunction->AddParameterBlock(3);
        parameterBlocks.push_back(&parameters[24 + i*6 + 3]);
        costFunction->AddParameterBlock(3);
      }
      costFunction->SetNumResiduals(4);
      problem.AddResidualBlock(costFunction,
                               NULL,
                               parameterBlocks
                              );
    }
  }
/*
  problem.SetParameterBlockConstant(&parameters[0]);
  problem.SetParameterBlockConstant(&parameters[4]);
  problem.SetParameterBlockConstant(&parameters[9]);
  problem.SetParameterBlockConstant(&parameters[13]);
  problem.SetParameterBlockConstant(&parameters[18]);
  problem.SetParameterBlockConstant(&parameters[21]);
*/

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

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

  delete [] parameters;

  return 0;
}

} // end namespace
