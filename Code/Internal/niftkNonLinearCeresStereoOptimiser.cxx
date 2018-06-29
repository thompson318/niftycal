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
#include <cmath>

#include <gsl/gsl_math.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_siman.h>
#include <gsl/gsl_ieee_utils.h>

#define SA_N_TRIES 12000           /* how many points do we try before stepping */
#define SA_ITERS_FIXED_T 1000      /* how many iterations for each T? */
#define SA_STEP_SIZE 1.0           /* max step size in random walk */
#define SA_K 1.0                   /* Boltzmann constant */
#define SA_T_INITIAL 5000.0        /* initial temperature */
#define SA_MU_T 1.002              /* damping factor for temperature */
#define SA_T_MIN 5.0e-1

gsl_siman_params_t params = {SA_N_TRIES, SA_ITERS_FIXED_T, SA_STEP_SIZE,
                             SA_K, SA_T_INITIAL, SA_MU_T, SA_T_MIN};

unsigned int totalNumberOfPoints;
unsigned int totalNumberOfParameters;
std::vector<std::vector<cv::Vec3f> > modelPoints;
std::vector<std::vector<cv::Vec2f> > leftImagePoints;
std::vector<std::vector<cv::Vec2f> > rightImagePoints;

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
      ceres::AngleAxisRotatePoint(&((parameters[4])[0]), m, l);
      l[0] += (parameters[4])[3];
      l[1] += (parameters[4])[4];
      l[2] += (parameters[4])[5];

      T r[3];
      ceres::AngleAxisRotatePoint(parameters[2], l, r);
      r[0] += (parameters[3])[0];
      r[1] += (parameters[3])[1];
      r[2] += (parameters[3])[2];

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

struct StereoProjectionConstraint {
public:
  StereoProjectionConstraint(const std::vector<cv::Vec3f>& m,
                             const std::vector<cv::Vec2f>& l,
                             const std::vector<cv::Vec2f>& r
                           )
  : m_ModelPoints(m)
  , m_LeftImagePoints(l)
  , m_RightImagePoints(r)
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
      ceres::QuaternionRotatePoint(&((parameters[6])[0]), m, l);
      l[0] += (parameters[7])[0];
      l[1] += (parameters[7])[1];
      l[2] += (parameters[7])[2];

      T lx;
      T ly;
      ProjectToPoint<T>(parameters[0],
                        parameters[1],
                        l,
                        lx,
                        ly
                       );

      T r[3];
      ceres::QuaternionRotatePoint(parameters[4], l, r);
      r[0] += (parameters[5])[0];
      r[1] += (parameters[5])[1];
      r[2] += (parameters[5])[2];

      T rx;
      T ry;
      ProjectToPoint<T>(parameters[2],
                        parameters[3],
                        r,
                        rx,
                        ry
                       );

      residuals[residualCounter++] = lx - T(static_cast<double>(m_LeftImagePoints[p](0)));
      residuals[residualCounter++] = ly - T(static_cast<double>(m_LeftImagePoints[p](1)));
      residuals[residualCounter++] = rx - T(static_cast<double>(m_RightImagePoints[p](0)));
      residuals[residualCounter++] = ry - T(static_cast<double>(m_RightImagePoints[p](1)));
    }
    return true;
  }

  std::vector<cv::Vec3f> m_ModelPoints;
  std::vector<cv::Vec2f> m_LeftImagePoints;
  std::vector<cv::Vec2f> m_RightImagePoints;
};

/*
void P1(void *xp)
{
  std::cerr << " ";
  for (unsigned int i = 0; i < totalNumberOfParameters; i++)
  {
    std::cerr << (reinterpret_cast<double*>(xp))[i] << " ";
  }
  std::cerr << std::endl;
}

double E1(void *xp)
{
  double rms = 0;

  double *parameters = reinterpret_cast<double*>(xp);
  unsigned long long int counter = 0;
  for (unsigned long int i = 0; i < modelPoints.size(); i++)
  {
    for (unsigned long int p = 0; p < modelPoints[i].size(); p++)
    {
      double m[3];
      m[0] = modelPoints[i][p](0);
      m[1] = modelPoints[i][p](1);
      m[2] = modelPoints[i][p](2);

      double l[3];
      ceres::QuaternionRotatePoint(&(parameters[25 + i*7]), m, l);
      l[0] += parameters[25 + i*7 + 4];
      l[1] += parameters[25 + i*7 + 5];
      l[2] += parameters[25 + i*7 + 6];

      double lx;
      double ly;
      ProjectToPoint<double>(&parameters[0],
                             &parameters[4],
                             l,
                             lx,
                             ly
                            );

      double r[3];
      ceres::QuaternionRotatePoint(&parameters[18], l, r);
      r[0] += parameters[18 + 4];
      r[1] += parameters[18 + 5];
      r[2] += parameters[18 + 6];

      double rx;
      double ry;
      ProjectToPoint<double>(&parameters[9],
                             &parameters[13],
                             r,
                             rx,
                             ry
                            );

      rms += (lx - leftImagePoints[i][p](0))*(lx - leftImagePoints[i][p](0));
      rms += (ly - leftImagePoints[i][p](1))*(ly - leftImagePoints[i][p](1));
      counter++;
      rms += (rx - rightImagePoints[i][p](0))*(rx - rightImagePoints[i][p](0));
      rms += (ry - rightImagePoints[i][p](1))*(ry - rightImagePoints[i][p](1));
      counter++;
    }
  }
  return rms / static_cast<double>(counter);
}

double M1(void *xp, void *yp)
{
  double m1 = 0;
  for (unsigned int i = 0; i < totalNumberOfParameters; i++)
  {
    m1 += std::fabs((reinterpret_cast<double*>(xp))[i]- (reinterpret_cast<double*>(yp))[i]);
  }
  return m1;
}

void S1(const gsl_rng * r, void *xp, double step_size)
{
  double *old_x = new double[totalNumberOfParameters];
  double *new_x = new double[totalNumberOfParameters];
  double *scales = new double[totalNumberOfParameters];

  scales[0] = 50;
  scales[1] = 50;
  scales[2] = 5;
  scales[3] = 5;
  scales[4] = 0.01;
  scales[5] = 0.01;
  scales[6] = 0.01;
  scales[7] = 0.01;
  scales[8] = 0.01;
  scales[9] = 50;
  scales[10] = 50;
  scales[11] = 5;
  scales[12] = 5;
  scales[13] = 0.01;
  scales[14] = 0.01;
  scales[15] = 0.01;
  scales[16] = 0.01;
  scales[17] = 0.01;
  scales[18] = 0.1;
  scales[19] = 0.1;
  scales[20] = 0.1;
  scales[21] = 0.1;
  scales[22] = 5;
  scales[23] = 5;
  scales[24] = 5;
  for (unsigned int i = 0; i < modelPoints.size(); i++)
  {
    scales[25 + i*7 + 0] = 0.1;
    scales[25 + i*7 + 1] = 0.1;
    scales[25 + i*7 + 2] = 0.1;
    scales[25 + i*7 + 3] = 0.1;
    scales[25 + i*7 + 4] = 5;
    scales[25 + i*7 + 5] = 5;
    scales[25 + i*7 + 6] = 5;
  }
  for (unsigned int i = 0; i < totalNumberOfParameters; i++)
  {
    double u = gsl_rng_uniform(r);
    old_x[i] = (reinterpret_cast<double*>(xp))[i];
    new_x[i] = u * 2 * step_size * scales[i] - step_size * scales[i]  + old_x[i];
  }
  memcpy(xp, new_x, totalNumberOfParameters * sizeof(double));

  delete [] old_x;
  delete [] new_x;
  delete [] scales;
}
*/
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
  totalNumberOfPoints = numberOfPoints;

  const unsigned int numberOfParameters = 4 // intrinsic left
                                        + 5 // distortion left
                                        + 4 // intrinsic right
                                        + 5 // distortion right
                                        + 7 // left to right transform
                                        + 7*rvecsLeft.size(); // extrinsics for each left-hand camera
  totalNumberOfParameters = numberOfParameters;

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

  double axisAngle[3];
  axisAngle[0] = leftToRightRotationVector.at<double>(0, 0);
  axisAngle[1] = leftToRightRotationVector.at<double>(0, 1);
  axisAngle[2] = leftToRightRotationVector.at<double>(0, 2);

  double quaternion[4];
  ceres::AngleAxisToQuaternion(axisAngle, quaternion);

  parameters[parameterCounter++] = quaternion[0];
  parameters[parameterCounter++] = quaternion[1];
  parameters[parameterCounter++] = quaternion[2];
  parameters[parameterCounter++] = quaternion[3];
  parameters[parameterCounter++] = leftToRightTranslationVector.at<double>(0, 0);
  parameters[parameterCounter++] = leftToRightTranslationVector.at<double>(0, 1);
  parameters[parameterCounter++] = leftToRightTranslationVector.at<double>(0, 2);

  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    axisAngle[0] = rvecsLeft[i].at<double>(0, 0);
    axisAngle[1] = rvecsLeft[i].at<double>(0, 1);
    axisAngle[2] = rvecsLeft[i].at<double>(0, 2);

    ceres::AngleAxisToQuaternion(axisAngle, quaternion);

    parameters[parameterCounter++] = quaternion[0];
    parameters[parameterCounter++] = quaternion[1];
    parameters[parameterCounter++] = quaternion[2];
    parameters[parameterCounter++] = quaternion[3];
    parameters[parameterCounter++] = tvecsLeft[i].at<double>(0, 0);
    parameters[parameterCounter++] = tvecsLeft[i].at<double>(0, 1);
    parameters[parameterCounter++] = tvecsLeft[i].at<double>(0, 2);
  }

  double *initialParameters = new double[numberOfParameters];
  for (unsigned int i = 0; i < numberOfParameters; i++)
  {
    initialParameters[i] = parameters[i];
  }

  modelPoints = objectVectors3D;
  leftImagePoints = leftVectors2D;
  rightImagePoints = rightVectors2D;

  /*
  const gsl_rng_type * T;
  gsl_rng * r;
  gsl_rng_env_setup();
  T = gsl_rng_default;
  r = gsl_rng_alloc(T);
  gsl_siman_solve(r, parameters, E1, S1, M1, NULL,
                    NULL, NULL, NULL,
                    numberOfParameters * sizeof(double), params);
  */

/*
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Problem problem;
*/
  // Cost functions for each left hand camera.
/*
  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    ceres::DynamicAutoDiffCostFunction<StereoProjectionConstraint> *stereoCostFunction =
      new ceres::DynamicAutoDiffCostFunction<StereoProjectionConstraint>(
        new StereoProjectionConstraint(objectVectors3D[i], leftVectors2D[i], rightVectors2D[i])
        );
    std::vector<double*> stereoBlocks;
    stereoBlocks.push_back(&parameters[0]);
    stereoCostFunction->AddParameterBlock(4);
    stereoBlocks.push_back(&parameters[4]);
    stereoCostFunction->AddParameterBlock(5);
    stereoBlocks.push_back(&parameters[9]);
    stereoCostFunction->AddParameterBlock(4);
    stereoBlocks.push_back(&parameters[13]);
    stereoCostFunction->AddParameterBlock(5);
    stereoBlocks.push_back(&parameters[18]);
    stereoCostFunction->AddParameterBlock(4);
    stereoBlocks.push_back(&parameters[22]);
    stereoCostFunction->AddParameterBlock(3);
    stereoBlocks.push_back(&parameters[25 + i*7]);
    stereoCostFunction->AddParameterBlock(4);
    stereoBlocks.push_back(&parameters[25 + i*7 + 4]);
    stereoCostFunction->AddParameterBlock(3);
    stereoCostFunction->SetNumResiduals(4 * objectVectors3D[i].size());

    problem.AddResidualBlock(stereoCostFunction,
                             NULL,
                             stereoBlocks
                            );

    problem.SetParameterBlockConstant(&parameters[0]);
    problem.SetParameterBlockConstant(&parameters[4]);
    problem.SetParameterBlockConstant(&parameters[9]);
    problem.SetParameterBlockConstant(&parameters[13]);
*/
    /*
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
    */

    /*
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
    rightCostFunction->AddParameterBlock(3);
    rightBlocks.push_back(&parameters[21]);
    rightCostFunction->AddParameterBlock(3);
    rightBlocks.push_back(&parameters[24 + i*6]);
    rightCostFunction->AddParameterBlock(6);
    rightCostFunction->SetNumResiduals(2 * objectVectors3D[i].size());

    problem.AddResidualBlock(rightCostFunction,
                             NULL,
                             rightBlocks
                            );

    problem.SetParameterBlockConstant(&parameters[9]);
    problem.SetParameterBlockConstant(&parameters[13]);
    //problem.SetParameterBlockConstant(&parameters[18]);
    */
/*
  }

  // Run the solver!
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
*/

  for (unsigned int i = 0; i < numberOfParameters; i++)
  {
    std::cerr << "Matt, post optimisation, i=" << i << ", i=" << initialParameters[i] << ", f=" << parameters[i] << ", d=" << parameters[i] - initialParameters[i] << std::endl;
  }

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

  quaternion[0] = parameters[parameterCounter++];
  quaternion[1] = parameters[parameterCounter++];
  quaternion[2] = parameters[parameterCounter++];
  quaternion[3] = parameters[parameterCounter++];
  ceres::QuaternionToAngleAxis(quaternion, axisAngle);

  leftToRightRotationVector.at<double>(0, 0) = axisAngle[0];
  leftToRightRotationVector.at<double>(0, 1) = axisAngle[1];
  leftToRightRotationVector.at<double>(0, 2) = axisAngle[2];

  cv::Rodrigues(leftToRightRotationVector, leftToRightRotationMatrix);

  leftToRightTranslationVector.at<double>(0, 0) = parameters[parameterCounter++];
  leftToRightTranslationVector.at<double>(0, 1) = parameters[parameterCounter++];
  leftToRightTranslationVector.at<double>(0, 2) = parameters[parameterCounter++];

  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    quaternion[0] = parameters[parameterCounter++];
    quaternion[1] = parameters[parameterCounter++];
    quaternion[2] = parameters[parameterCounter++];
    quaternion[3] = parameters[parameterCounter++];
    ceres::QuaternionToAngleAxis(quaternion, axisAngle);

    rvecsLeft[i].at<double>(0, 0) = axisAngle[0];
    rvecsLeft[i].at<double>(0, 1) = axisAngle[1];
    rvecsLeft[i].at<double>(0, 2) = axisAngle[2];
    tvecsLeft[i].at<double>(0, 0) = parameters[parameterCounter++];
    tvecsLeft[i].at<double>(0, 1) = parameters[parameterCounter++];
    tvecsLeft[i].at<double>(0, 2) = parameters[parameterCounter++];
  }

  delete [] parameters;
  delete [] initialParameters;

  return 0;
}

} // end namespace
