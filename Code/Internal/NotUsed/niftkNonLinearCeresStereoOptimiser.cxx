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
  StereoProjectionConstraint(const cv::Vec3f& m,
                             const cv::Vec2f& l,
                             const cv::Vec2f& r
                           )
  : m_ModelPoint(m)
  , m_LeftImagePoint(l)
  , m_RightImagePoint(r)
  {
  }

  template <typename T>
/*
  bool operator()(T const* const* parameters,
                  T* residuals) const {
*/
  bool operator()(const T* const leftIntrinsic,
                  const T* const leftDistortion,
                  const T* const rightIntrinsic,
                  const T* const rightDistortion,
                  const T* const baselineRotation,
                  const T* const baselineTranslation,
                  const T* const leftRotation,
                  const T* const leftTranslation,
                  const T* const rightRotation,
                  const T* const rightTranslation,
                  T* residuals) const {

    unsigned long long int residualCounter = 0;
    T m[3];
    m[0] = T(static_cast<double>(m_ModelPoint(0)));
    m[1] = T(static_cast<double>(m_ModelPoint(1)));
    m[2] = T(static_cast<double>(m_ModelPoint(2)));

    T b[3];
    ceres::QuaternionRotatePoint(baselineRotation, m, b);

    T bt[3];
    bt[0] = b[0] + baselineTranslation[0];
    bt[1] = b[1] + baselineTranslation[1];
    bt[2] = b[2] + baselineTranslation[2];

    T l[3];
    ceres::QuaternionRotatePoint(leftRotation, bt, l);

    T lt[3];
    lt[0] = l[0] + leftTranslation[0];
    lt[1] = l[1] + leftTranslation[1];
    lt[2] = l[2] + leftTranslation[2];

    T lx;
    T ly;
    ProjectToPoint<T>(leftIntrinsic,
                      leftDistortion,
                      lt,
                      lx,
                      ly
                     );

    T r[3];
    ceres::QuaternionRotatePoint(rightRotation, bt, r);

    T rt[3];
    rt[0] = r[0] + rightTranslation[0];
    rt[1] = r[1] + rightTranslation[1];
    rt[2] = r[2] + rightTranslation[2];

    T rx;
    T ry;
    ProjectToPoint<T>(rightIntrinsic,
                      rightDistortion,
                      rt,
                      rx,
                      ry
                     );

    residuals[residualCounter++] = (lx - T(static_cast<double>(m_LeftImagePoint(0))));
    residuals[residualCounter++] = (ly - T(static_cast<double>(m_LeftImagePoint(1))));
    residuals[residualCounter++] = (rx - T(static_cast<double>(m_RightImagePoint(0))));
    residuals[residualCounter++] = (ry - T(static_cast<double>(m_RightImagePoint(1))));

/*
    residuals[residualCounter++] =
        sqrt((lx - T(static_cast<double>(m_LeftImagePoints[i][p](0))))*(lx - T(static_cast<double>(m_LeftImagePoints[i][p](0))))
            +(ly - T(static_cast<double>(m_LeftImagePoints[i][p](1))))*(ly - T(static_cast<double>(m_LeftImagePoints[i][p](1))))
            );
    residuals[residualCounter++] =
       sqrt((rx - T(static_cast<double>(m_RightImagePoints[i][p](0))))*(rx - T(static_cast<double>(m_RightImagePoints[i][p](0))))
           +(ry - T(static_cast<double>(m_RightImagePoints[i][p](1))))*(ry - T(static_cast<double>(m_RightImagePoints[i][p](1))))
           );
*/
/*
    for (std::vector<std::vector<cv::Vec3f> >::size_type i = 0; i < m_ModelPoints.size(); i++)
    {
      for (std::vector<cv::Vec3f>::size_type p = 0; p < m_ModelPoints[i].size(); p++)
      {
*/

/*
        residuals[residualCounter++] = lx - T(static_cast<double>(m_LeftImagePoints[i][p](0)));
        residuals[residualCounter++] = ly - T(static_cast<double>(m_LeftImagePoints[i][p](1)));
        residuals[residualCounter++] = rx - T(static_cast<double>(m_RightImagePoints[i][p](0)));
        residuals[residualCounter++] = ry - T(static_cast<double>(m_RightImagePoints[i][p](1)));
        */
/*
        std::cerr << "i=" << i << ", p=" << p
                  << ", ldx=" <<  lx - T(static_cast<double>(m_LeftImagePoints[i][p](0)))
                  << ", ldy=" <<  ly - T(static_cast<double>(m_LeftImagePoints[i][p](1)))
                  << ", rdx=" <<  rx - T(static_cast<double>(m_RightImagePoints[i][p](0)))
                  << ", rdy=" <<  ry - T(static_cast<double>(m_RightImagePoints[i][p](1)))
                                                             << std::endl
                                                             ;
*/
/*
      }
    }
*/
    return true;
  }

  cv::Vec3f m_ModelPoint;
  cv::Vec2f m_LeftImagePoint;
  cv::Vec2f m_RightImagePoint;
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
                                  std::vector<cv::Mat>& rvecsRight,
                                  std::vector<cv::Mat>& tvecsRight,
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
                                        + 4 * objectVectors3D.size() // base frame rotation
                                        + 3 * objectVectors3D.size() // base frame translation
                                        + 4 // left camera rotation
                                        + 3 // left camera translation
                                        + 4 // right camera rotation
                                        + 3 // right camera translation
                                        ;

  std::cerr << "Matt, numberOfParameters = " << numberOfParameters << std::endl;

  totalNumberOfParameters = numberOfParameters;
//  leftToRightRotationMatrix = cv::Mat::eye(3, 3, CV_64FC1);
//  leftToRightTranslationVector = cv::Mat::zeros(1, 3, CV_64FC1);

  double *parameters = new double[numberOfParameters];

  double axisAngle[3];
  double quaternion[4];

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

  std::list<cv::Matx44d> toLeftCameraExtrinsics;
  std::list<cv::Matx44d> toRightCameraExtrinsics;
  cv::Mat rvec = cv::Mat::zeros(1, 3, CV_64FC1);
  cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64FC1);

  // Base frame rotation (4) and translation (3), in that order.
  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    cv::Matx44d leftExtrinsic = niftk::RodriguesToMatrix(rvecsLeft[i], tvecsLeft[i]);
    cv::Matx44d rightExtrinsic = niftk::RodriguesToMatrix(rvecsRight[i], tvecsRight[i]);

    std::list<cv::Matx44d> toAverage;
    toAverage.push_back(leftExtrinsic);
    toAverage.push_back(rightExtrinsic);

    cv::Matx44d averageExtrinsic = niftk::AverageMatricesUsingEigenValues(toAverage);

    toLeftCameraExtrinsics.push_back(leftExtrinsic * averageExtrinsic.inv());
    toRightCameraExtrinsics.push_back(rightExtrinsic * averageExtrinsic.inv());

    niftk::MatrixToRodrigues(averageExtrinsic, rvec, tvec);

    axisAngle[0] = rvec.at<double>(0, 0);
    axisAngle[1] = rvec.at<double>(0, 1);
    axisAngle[2] = rvec.at<double>(0, 2);

    ceres::AngleAxisToQuaternion(axisAngle, quaternion);

    parameters[parameterCounter++] = quaternion[0];
    parameters[parameterCounter++] = quaternion[1];
    parameters[parameterCounter++] = quaternion[2];
    parameters[parameterCounter++] = quaternion[3];

    parameters[parameterCounter++] = tvec.at<double>(0, 0);
    parameters[parameterCounter++] = tvec.at<double>(0, 1);
    parameters[parameterCounter++] = tvec.at<double>(0, 2);
  }

  cv::Matx44d toLeftCamera = niftk::AverageMatricesUsingEigenValues(toLeftCameraExtrinsics);
  cv::Matx44d toRightCamera = niftk::AverageMatricesUsingEigenValues(toRightCameraExtrinsics);

  // From baseline to left camera
  niftk::MatrixToRodrigues(toLeftCamera, rvec, tvec);
  axisAngle[0] = rvec.at<double>(0, 0);
  axisAngle[1] = rvec.at<double>(0, 1);
  axisAngle[2] = rvec.at<double>(0, 2);

  ceres::AngleAxisToQuaternion(axisAngle, quaternion);

  parameters[parameterCounter++] = quaternion[0];
  parameters[parameterCounter++] = quaternion[1];
  parameters[parameterCounter++] = quaternion[2];
  parameters[parameterCounter++] = quaternion[3];

  parameters[parameterCounter++] = tvec.at<double>(0, 0);
  parameters[parameterCounter++] = tvec.at<double>(0, 1);
  parameters[parameterCounter++] = tvec.at<double>(0, 2);

  // From baseline to right camera
  niftk::MatrixToRodrigues(toRightCamera, rvec, tvec);
  axisAngle[0] = rvec.at<double>(0, 0);
  axisAngle[1] = rvec.at<double>(0, 1);
  axisAngle[2] = rvec.at<double>(0, 2);

  ceres::AngleAxisToQuaternion(axisAngle, quaternion);

  parameters[parameterCounter++] = quaternion[0];
  parameters[parameterCounter++] = quaternion[1];
  parameters[parameterCounter++] = quaternion[2];
  parameters[parameterCounter++] = quaternion[3];

  parameters[parameterCounter++] = tvec.at<double>(0, 0);
  parameters[parameterCounter++] = tvec.at<double>(0, 1);
  parameters[parameterCounter++] = tvec.at<double>(0, 2);

/*
  // Left camera rotation, initialise to identity.
  axisAngle[0] = 0;
  axisAngle[1] = 0;
  axisAngle[2] = 0;
  ceres::AngleAxisToQuaternion(axisAngle, quaternion);
  parameters[parameterCounter++] = quaternion[0];
  parameters[parameterCounter++] = quaternion[1];
  parameters[parameterCounter++] = quaternion[2];
  parameters[parameterCounter++] = quaternion[3];

  // Right camera position
  cv::Mat leftToRightRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  cv::Rodrigues(leftToRightRotationMatrix, leftToRightRotationVector);
  cv::Matx44d leftToRight = niftk::RodriguesToMatrix(leftToRightRotationVector, leftToRightTranslationVector);
  cv::Matx44d rightToLeft = leftToRight.inv();
  cv::Matx41d origin;
  origin(0, 0) = 0;
  origin(1, 0) = 0;
  origin(2, 0) = 0;
  origin(3, 0) = 1;
  cv::Matx41d rightCameraPosition = rightToLeft * origin;
  rightCameraPosition(1, 0) = 0;
  rightCameraPosition(2, 0) = 0;
  parameters[parameterCounter++] = rightCameraPosition(0, 0);
  parameters[parameterCounter++] = rightCameraPosition(1, 0);
  //parameters[parameterCounter++] = rightCameraPosition(2, 0); // Do not allow z offset.

  // Right camera rotation.
  axisAngle[0] = leftToRightRotationVector.at<double>(0, 0);
  axisAngle[1] = leftToRightRotationVector.at<double>(0, 1);
  axisAngle[2] = leftToRightRotationVector.at<double>(0, 2);
  ceres::AngleAxisToQuaternion(axisAngle, quaternion);
  parameters[parameterCounter++] = quaternion[0];
  parameters[parameterCounter++] = quaternion[1];
  parameters[parameterCounter++] = quaternion[2];
  parameters[parameterCounter++] = quaternion[3];

  // Base frame rotation = left camera rotation.
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
  }

  // Base frame translation = left camera position.
  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    cv::Matx44d worldToCamera = niftk::RodriguesToMatrix(rvecsLeft[i], tvecsLeft[i]);
    cv::Matx44d cameraToWorld = worldToCamera.inv();
    cv::Matx41d origin;
    origin(0, 0) = 0;
    origin(1, 0) = 0;
    origin(2, 0) = 0;
    origin(3, 0) = 1;
    cv::Matx41d cameraPosition = cameraToWorld * origin;
    parameters[parameterCounter++] = cameraPosition(0, 0);
    parameters[parameterCounter++] = cameraPosition(1, 0);
    parameters[parameterCounter++] = cameraPosition(2, 0);
  }
*/

  double *initialParameters = new double[numberOfParameters];
  for (unsigned int i = 0; i < numberOfParameters; i++)
  {
    initialParameters[i] = parameters[i];
  }

  /*
  modelPoints = objectVectors3D;
  leftImagePoints = leftVectors2D;
  rightImagePoints = rightVectors2D;


  const gsl_rng_type * T;
  gsl_rng * r;
  gsl_rng_env_setup();
  T = gsl_rng_default;
  r = gsl_rng_alloc(T);
  gsl_siman_solve(r, parameters, E1, S1, M1, NULL,
                    NULL, NULL, NULL,
                    numberOfParameters * sizeof(double), params);
  */

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.gradient_tolerance = 1e-16;
  options.function_tolerance = 1e-16;

  ceres::Problem problem;

  for (unsigned int i = 0; i < objectVectors3D.size(); i++)
  {
    for (unsigned int p = 0; p < objectVectors3D[i].size(); p++)
    {
      ceres::AutoDiffCostFunction<StereoProjectionConstraint,
          4, // residuals
          4, // left intrinsic
          5, // left distortion
          4, // right intrinsic
          5, // right distortion
          4, // baseline rotation
          3, // baseline translation
          4, // left rotation
          3, // left translation
          4, // right rotation
          3  // right translation
          > *stereoCostFunction =
        new ceres::AutoDiffCostFunction<StereoProjectionConstraint, 4, 4, 5, 4, 5, 4, 3, 4, 3, 4, 3>(
          new StereoProjectionConstraint(objectVectors3D[i][p], leftVectors2D[i][p], rightVectors2D[i][p])
          );
/*
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
      stereoCostFunction->AddParameterBlock(4); // left rotation
      stereoBlocks.push_back(&parameters[22]);
      stereoCostFunction->AddParameterBlock(2); // right position
      stereoBlocks.push_back(&parameters[24]);
      stereoCostFunction->AddParameterBlock(4);
      stereoBlocks.push_back(&parameters[28 + i*4]);
      stereoCostFunction->AddParameterBlock(4);
      stereoBlocks.push_back(&parameters[28 + rvecsLeft.size()*4 + i*3]);
      stereoCostFunction->AddParameterBlock(3);
      stereoCostFunction->SetNumResiduals(4);
*/
      problem.AddResidualBlock(stereoCostFunction,
                               NULL,
                               &parameters[0],  // left intrinsic
                               &parameters[4],  // left distortion
                               &parameters[9],  // right intrinsic
                               &parameters[13], // right distortion
                               &parameters[18 + i*7 + 0], // baseline rotation
                               &parameters[18 + i*7 + 4], // baseline translation
                               &parameters[18 + rvecsLeft.size()*7 + 0], // left rotation
                               &parameters[18 + rvecsLeft.size()*7 + 4], // left translation
                               &parameters[18 + rvecsLeft.size()*7 + 7], // right rotation
                               &parameters[18 + rvecsLeft.size()*7 + 11] // right translation
                              );
      problem.SetParameterBlockConstant(&parameters[18 + i*7 + 0]);
    }
  }

  std::cerr << "Matt, after loop, num parameters=" << 18 + rvecsLeft.size()*7 + 11 + 3 << std::endl;

/*
  problem.SetParameterLowerBound(&(parameters[0]), 0, parameters[0] - 50);
  problem.SetParameterUpperBound(&(parameters[0]), 0, parameters[0] + 50);
  problem.SetParameterLowerBound(&(parameters[0]), 1, parameters[1] - 50);
  problem.SetParameterUpperBound(&(parameters[0]), 1, parameters[1] + 50);
  problem.SetParameterLowerBound(&(parameters[0]), 2, parameters[2] - 10);
  problem.SetParameterUpperBound(&(parameters[0]), 2, parameters[2] + 10);
  problem.SetParameterLowerBound(&(parameters[0]), 3, parameters[3] - 10);
  problem.SetParameterUpperBound(&(parameters[0]), 3, parameters[3] + 10);
  problem.SetParameterLowerBound(&(parameters[4]), 0, parameters[4] - 0.1);
  problem.SetParameterUpperBound(&(parameters[4]), 0, parameters[4] + 0.1);
  problem.SetParameterLowerBound(&(parameters[4]), 1, parameters[5] - 0.1);
  problem.SetParameterUpperBound(&(parameters[4]), 1, parameters[5] + 0.1);
  problem.SetParameterLowerBound(&(parameters[4]), 2, parameters[6] - 0.1);
  problem.SetParameterUpperBound(&(parameters[4]), 2, parameters[6] + 0.1);
  problem.SetParameterLowerBound(&(parameters[4]), 3, parameters[7] - 0.1);
  problem.SetParameterUpperBound(&(parameters[4]), 3, parameters[7] + 0.1);
  problem.SetParameterLowerBound(&(parameters[4]), 4, parameters[8] - 0.1);
  problem.SetParameterUpperBound(&(parameters[4]), 4, parameters[8] + 0.1);
  problem.SetParameterLowerBound(&(parameters[9]), 0, parameters[9] - 50);
  problem.SetParameterUpperBound(&(parameters[9]), 0, parameters[9] + 50);
  problem.SetParameterLowerBound(&(parameters[9]), 1, parameters[10] - 50);
  problem.SetParameterUpperBound(&(parameters[9]), 1, parameters[10] + 50);
  problem.SetParameterLowerBound(&(parameters[9]), 2, parameters[11] - 10);
  problem.SetParameterUpperBound(&(parameters[9]), 2, parameters[11] + 10);
  problem.SetParameterLowerBound(&(parameters[9]), 3, parameters[12] - 10);
  problem.SetParameterUpperBound(&(parameters[9]), 3, parameters[12] + 10);
  problem.SetParameterLowerBound(&(parameters[13]), 0, parameters[13] - 0.1);
  problem.SetParameterUpperBound(&(parameters[13]), 0, parameters[13] + 0.1);
  problem.SetParameterLowerBound(&(parameters[13]), 1, parameters[14] - 0.1);
  problem.SetParameterUpperBound(&(parameters[13]), 1, parameters[14] + 0.1);
  problem.SetParameterLowerBound(&(parameters[13]), 2, parameters[15] - 0.1);
  problem.SetParameterUpperBound(&(parameters[13]), 2, parameters[15] + 0.1);
  problem.SetParameterLowerBound(&(parameters[13]), 3, parameters[16] - 0.1);
  problem.SetParameterUpperBound(&(parameters[13]), 3, parameters[16] + 0.1);
  problem.SetParameterLowerBound(&(parameters[13]), 4, parameters[17] - 0.1);
  problem.SetParameterUpperBound(&(parameters[13]), 4, parameters[17] + 0.1);

  // Left rotation
  problem.SetParameterLowerBound(&(parameters[18]), 0, parameters[18] - 0.1);
  problem.SetParameterUpperBound(&(parameters[18]), 0, parameters[18] + 0.1);
  problem.SetParameterLowerBound(&(parameters[18]), 1, parameters[19] - 0.1);
  problem.SetParameterUpperBound(&(parameters[18]), 1, parameters[19] + 0.1);
  problem.SetParameterLowerBound(&(parameters[18]), 2, parameters[20] - 0.1);
  problem.SetParameterUpperBound(&(parameters[18]), 2, parameters[20] + 0.1);
  problem.SetParameterLowerBound(&(parameters[18]), 3, parameters[21] - 0.1);
  problem.SetParameterUpperBound(&(parameters[18]), 3, parameters[21] + 0.1);

  // Right position
  problem.SetParameterLowerBound(&(parameters[22]), 0, parameters[22] - 0.5);
  problem.SetParameterUpperBound(&(parameters[22]), 0, parameters[22] + 0.5);
  problem.SetParameterLowerBound(&(parameters[22]), 1, parameters[23] - 0.1);
  problem.SetParameterUpperBound(&(parameters[22]), 1, parameters[23] + 0.1);

  // Right rotation
  problem.SetParameterLowerBound(&(parameters[24]), 0, parameters[24] - 0.1);
  problem.SetParameterUpperBound(&(parameters[24]), 0, parameters[24] + 0.1);
  problem.SetParameterLowerBound(&(parameters[24]), 1, parameters[25] - 0.1);
  problem.SetParameterUpperBound(&(parameters[24]), 1, parameters[25] + 0.1);
  problem.SetParameterLowerBound(&(parameters[24]), 2, parameters[26] - 0.1);
  problem.SetParameterUpperBound(&(parameters[24]), 2, parameters[26] + 0.1);
  problem.SetParameterLowerBound(&(parameters[24]), 3, parameters[27] - 0.1);
  problem.SetParameterUpperBound(&(parameters[24]), 3, parameters[27] + 0.1);

  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    problem.SetParameterLowerBound(&(parameters[28 + i*4]), 0, parameters[28 + i*4 + 0] - 0.1);
    problem.SetParameterUpperBound(&(parameters[28 + i*4]), 0, parameters[28 + i*4 + 0] + 0.1);
    problem.SetParameterLowerBound(&(parameters[28 + i*4]), 1, parameters[28 + i*4 + 1] - 0.1);
    problem.SetParameterUpperBound(&(parameters[28 + i*4]), 1, parameters[28 + i*4 + 1] + 0.1);
    problem.SetParameterLowerBound(&(parameters[28 + i*4]), 2, parameters[28 + i*4 + 2] - 0.1);
    problem.SetParameterUpperBound(&(parameters[28 + i*4]), 2, parameters[28 + i*4 + 2] + 0.1);
    problem.SetParameterLowerBound(&(parameters[28 + i*4]), 3, parameters[28 + i*4 + 3] - 0.1);
    problem.SetParameterUpperBound(&(parameters[28 + i*4]), 3, parameters[28 + i*4 + 3] + 0.1);
  }

  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    problem.SetParameterLowerBound(&(parameters[28 + + rvecsLeft.size()*4 + i*3]), 0, parameters[28 + rvecsLeft.size()*4 + i*3 + 0] - 2);
    problem.SetParameterUpperBound(&(parameters[28 + + rvecsLeft.size()*4 + i*3]), 0, parameters[28 + rvecsLeft.size()*4 + i*3 + 0] + 2);
    problem.SetParameterLowerBound(&(parameters[28 + + rvecsLeft.size()*4 + i*3]), 1, parameters[28 + rvecsLeft.size()*4 + i*3 + 1] - 2);
    problem.SetParameterUpperBound(&(parameters[28 + + rvecsLeft.size()*4 + i*3]), 1, parameters[28 + rvecsLeft.size()*4 + i*3 + 1] + 2);
    problem.SetParameterLowerBound(&(parameters[28 + + rvecsLeft.size()*4 + i*3]), 2, parameters[28 + rvecsLeft.size()*4 + i*3 + 2] - 2);
    problem.SetParameterUpperBound(&(parameters[28 + + rvecsLeft.size()*4 + i*3]), 2, parameters[28 + rvecsLeft.size()*4 + i*3 + 2] + 2);
  }
  */

  //problem.SetParameterBlockConstant(&parameters[0]);
  //problem.SetParameterBlockConstant(&parameters[4]);
  //problem.SetParameterBlockConstant(&parameters[9]);
  //problem.SetParameterBlockConstant(&parameters[13]);

/*
  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
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
*/

  // Run the solver!
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  for (unsigned int i = 0; i < numberOfParameters; i++)
  {
    std::cerr << "Matt, post optimisation, i=" << i << ", b=" << initialParameters[i] << ", a=" << parameters[i] << ", d=" << parameters[i] - initialParameters[i] << std::endl;
  }
  std::cerr << "Matt, indicative error=" << std::sqrt(summary.final_cost / static_cast<double>(numberOfPoints)) << std::endl;
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

  // Extract common baseline to left
  quaternion[0] = parameters[18 + rvecsLeft.size()*7 + 0];
  quaternion[1] = parameters[18 + rvecsLeft.size()*7 + 1];
  quaternion[2] = parameters[18 + rvecsLeft.size()*7 + 2];
  quaternion[3] = parameters[18 + rvecsLeft.size()*7 + 3];
  ceres::QuaternionToAngleAxis(quaternion, axisAngle);
  rvec.at<double>(0, 0) = axisAngle[0];
  rvec.at<double>(0, 1) = axisAngle[1];
  rvec.at<double>(0, 2) = axisAngle[2];
  tvec.at<double>(0, 0) = parameters[18 + rvecsLeft.size()*7 + 4];
  tvec.at<double>(0, 1) = parameters[18 + rvecsLeft.size()*7 + 5];
  tvec.at<double>(0, 2) = parameters[18 + rvecsLeft.size()*7 + 6];
  cv::Matx44d baselineToLeft = niftk::RodriguesToMatrix(rvec, tvec);
  std::cerr << "Matt, baselineToLeft=" << baselineToLeft << std::endl;

  // Extract common baseline to right
  quaternion[0] = parameters[18 + rvecsLeft.size()*7 + 7];
  quaternion[1] = parameters[18 + rvecsLeft.size()*7 + 8];
  quaternion[2] = parameters[18 + rvecsLeft.size()*7 + 9];
  quaternion[3] = parameters[18 + rvecsLeft.size()*7 + 10];
  ceres::QuaternionToAngleAxis(quaternion, axisAngle);
  rvec.at<double>(0, 0) = axisAngle[0];
  rvec.at<double>(0, 1) = axisAngle[1];
  rvec.at<double>(0, 2) = axisAngle[2];
  tvec.at<double>(0, 0) = parameters[18 + rvecsLeft.size()*7 + 11];
  tvec.at<double>(0, 1) = parameters[18 + rvecsLeft.size()*7 + 12];
  tvec.at<double>(0, 2) = parameters[18 + rvecsLeft.size()*7 + 13];
  cv::Matx44d baselineToRight = niftk::RodriguesToMatrix(rvec, tvec);
  std::cerr << "Matt, baselineToRight=" << baselineToRight << std::endl;

  // Calculate final left to right transformation.
  cv::Matx44d leftToRight = baselineToRight * baselineToLeft.inv();
  niftk::MatrixToRodrigues(leftToRight, rvec, leftToRightTranslationVector);
  cv::Rodrigues(rvec, leftToRightRotationMatrix);
  std::cerr << "Matt, leftToRightTranslationVector=" << leftToRightTranslationVector << std::endl;
  std::cerr << "Matt, leftToRightRotationMatrix=" << leftToRightRotationMatrix << std::endl;

  // Now set rvecs and tvecs consistently
  // (consistent with respect to the left to right transformation)
  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    quaternion[0] = parameters[18 + i*7 + 0];
    quaternion[1] = parameters[18 + i*7 + 1];
    quaternion[2] = parameters[18 + i*7 + 2];
    quaternion[3] = parameters[18 + i*7 + 3];
    ceres::QuaternionToAngleAxis(quaternion, axisAngle);
    rvec.at<double>(0, 0) = axisAngle[0];
    rvec.at<double>(0, 1) = axisAngle[1];
    rvec.at<double>(0, 2) = axisAngle[2];
    tvec.at<double>(0, 0) = parameters[18 + i*7 + 4];
    tvec.at<double>(0, 1) = parameters[18 + i*7 + 5];
    tvec.at<double>(0, 2) = parameters[18 + i*7 + 6];
    cv::Matx44d baseline = niftk::RodriguesToMatrix(rvec, tvec);
    cv::Matx44d leftExtrinsic = baselineToLeft * baseline;
    niftk::MatrixToRodrigues(leftExtrinsic, rvecsLeft[i], tvecsLeft[i]);
    cv::Matx44d rightExtrinsic = baselineToRight * baseline;
    niftk::MatrixToRodrigues(rightExtrinsic, rvecsRight[i], tvecsRight[i]);
  }
/*
  // Extract R_L, left camera rotation
  quaternion[0] = parameters[parameterCounter++];
  quaternion[1] = parameters[parameterCounter++];
  quaternion[2] = parameters[parameterCounter++];
  quaternion[3] = parameters[parameterCounter++];
  ceres::QuaternionToAngleAxis(quaternion, axisAngle);
  cv::Mat leftRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  leftRotationVector.at<double>(0, 0) = axisAngle[0];
  leftRotationVector.at<double>(0, 1) = axisAngle[1];
  leftRotationVector.at<double>(0, 2) = axisAngle[2];

  // Extract C_R, right camera position
  cv::Matx31d cr;
  cr(0, 0) = parameters[parameterCounter++];
  cr(1, 0) = parameters[parameterCounter++];
  cr(2, 0) = 0; //parameters[parameterCounter++];

  // Extract R_R, right camera rotation
  quaternion[0] = parameters[parameterCounter++];
  quaternion[1] = parameters[parameterCounter++];
  quaternion[2] = parameters[parameterCounter++];
  quaternion[3] = parameters[parameterCounter++];
  ceres::QuaternionToAngleAxis(quaternion, axisAngle);
  leftToRightRotationVector.at<double>(0, 0) = axisAngle[0];
  leftToRightRotationVector.at<double>(0, 1) = axisAngle[1];
  leftToRightRotationVector.at<double>(0, 2) = axisAngle[2];

  // Re-compute the right camera offset.
  cv::Rodrigues(leftToRightRotationVector, leftToRightRotationMatrix);
  cv::Matx31d crTransformed = cv::Matx33d(leftToRightRotationMatrix) * cr;
  cv::Matx31d crNegated = -1 * crTransformed;
  leftToRightTranslationVector.at<double>(0, 0) = crNegated(0, 0);
  leftToRightTranslationVector.at<double>(0, 1) = crNegated(1, 0);
  leftToRightTranslationVector.at<double>(0, 2) = crNegated(2, 0);

  // Read out baseline left camera rotation, and store in rvecsLeft array temporarily.
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
  }

  // Read out baseline left camera translation, and store in tvecsLeft array temporarily.
  for (unsigned int i = 0; i < tvecsLeft.size(); i++)
  {
    tvecsLeft[i].at<double>(0, 0) = parameters[parameterCounter++];
    tvecsLeft[i].at<double>(0, 1) = parameters[parameterCounter++];
    tvecsLeft[i].at<double>(0, 2) = parameters[parameterCounter++];
  }

  // Compute final left hand translation and rotation.
  cv::Mat tmpRotationVector = cv::Mat::zeros(1, 3, CV_64FC1);
  cv::Mat tmpRotationMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Mat tmpTranslationVector = cv::Mat::zeros(3, 1, CV_64FC1);
  cv::Mat transformedTranslationVector = cv::Mat::zeros(3, 1, CV_64FC1);
  cv::Mat leftRotation = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Mat baselineRotation = cv::Mat::zeros(3, 3, CV_64FC1);
  cv::Mat baselineTransform = cv::Mat::eye(4, 4, CV_64FC1);
  cv::Mat leftTransform = cv::Mat::eye(4, 4, CV_64FC1);
  cv::Mat combinedTransform = cv::Mat::eye(4, 4, CV_64FC1);

  for (unsigned int i = 0; i < rvecsLeft.size(); i++)
  {
    cv::Rodrigues(rvecsLeft[i], baselineRotation);
    tmpTranslationVector.at<double>(0, 0) = tvecsLeft[i].at<double>(0, 0);
    tmpTranslationVector.at<double>(1, 0) = tvecsLeft[i].at<double>(0, 1);
    tmpTranslationVector.at<double>(2, 0) = tvecsLeft[i].at<double>(0, 2);
    transformedTranslationVector = baselineRotation * tmpTranslationVector;
    transformedTranslationVector.at<double>(0, 0) = -1.0 * transformedTranslationVector.at<double>(0, 0);
    transformedTranslationVector.at<double>(1, 0) = -1.0 * transformedTranslationVector.at<double>(1, 0);
    transformedTranslationVector.at<double>(2, 0) = -1.0 * transformedTranslationVector.at<double>(2, 0);
    baselineRotation.copyTo(baselineTransform(cv::Rect(0, 0, 3, 3)));
    baselineTransform.at<double>(0, 3) = transformedTranslationVector.at<double>(0, 0);
    baselineTransform.at<double>(1, 3) = transformedTranslationVector.at<double>(1, 0);
    baselineTransform.at<double>(2, 3) = transformedTranslationVector.at<double>(2, 0);

    tmpRotationVector.at<double>(0, 0) = leftRotationVector.at<double>(0, 0);
    tmpRotationVector.at<double>(0, 1) = leftRotationVector.at<double>(0, 1);
    tmpRotationVector.at<double>(0, 2) = leftRotationVector.at<double>(0, 2);
    cv::Rodrigues(tmpRotationVector, leftRotation);
    leftRotation.copyTo(leftTransform(cv::Rect(0, 0, 3, 3)));

    combinedTransform = leftTransform * baselineTransform;

    combinedTransform(cv::Rect(0,0,3,3)).copyTo(tmpRotationMatrix);
    cv::Rodrigues(tmpRotationMatrix, rvecsLeft[i]);

    tvecsLeft[i].at<double>(0, 0) = combinedTransform.at<double>(0, 3);
    tvecsLeft[i].at<double>(0, 1) = combinedTransform.at<double>(1, 3);
    tvecsLeft[i].at<double>(0, 2) = combinedTransform.at<double>(2, 3);
  }
*/
  delete [] parameters;
  delete [] initialParameters;

  return 0;
}

} // end namespace

