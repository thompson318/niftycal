/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkMatrixUtilities.h"
#include "niftkNiftyCalExceptionMacro.h"
#include <set>

namespace niftk {

//-----------------------------------------------------------------------------
cv::Matx44d RotationAndTranslationToMatrix(const cv::Mat& rotationMatrix3x3,
                                           const cv::Mat& translationVector3x1)
{
  if (rotationMatrix3x3.rows != 3 || rotationMatrix3x3.cols != 3)
  {
    niftkNiftyCalThrow() << "Invalid rotation matrix size.";
  }

  cv::Matx44d mat = cv::Matx44d::eye();

  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      mat(r,c) = rotationMatrix3x3.at<double>(r,c);
    }
    mat(r,3) = translationVector3x1.at<double>(r, 0);
  }
  return mat;
}


//-----------------------------------------------------------------------------
cv::Matx44d RodriguesToMatrix(const cv::Mat& rotationVector1x3,
                              const cv::Mat& translationVector1x3)
{
  cv::Mat rotationMatrix;
  cv::Rodrigues(rotationVector1x3, rotationMatrix);

  cv::Mat t = cvCreateMat(3, 1, CV_64FC1);
  t.at<double>(0, 0) = translationVector1x3.at<double>(0, 0);
  t.at<double>(1, 0) = translationVector1x3.at<double>(0, 1);
  t.at<double>(2, 0) = translationVector1x3.at<double>(0, 2);

  return RotationAndTranslationToMatrix(rotationMatrix, t);
}


//-----------------------------------------------------------------------------
void MatrixToRodrigues(const cv::Matx44d& mat,
                       cv::Mat& rotationVector1x3,
                       cv::Mat& translationVector1x3)
{
  cv::Mat rotationMatrix = cvCreateMat(3, 3, CV_64FC1);
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      rotationMatrix.at<double>(r, c) = mat(r, c);
    }
    translationVector1x3.at<double>(0, r) = mat(r, 3);
  }
  cv::Rodrigues(rotationMatrix, rotationVector1x3);
}


//-----------------------------------------------------------------------------
cv::Matx14d RodriguesToAxisAngle(const cv::Mat& rotationVector1x3)
{
  cv::Matx14d axisAngle = cv::Matx14d::zeros();
  double norm = cv::norm(rotationVector1x3);
  axisAngle(0, 0) = rotationVector1x3.at<double>(0, 0) / norm;
  axisAngle(0, 1) = rotationVector1x3.at<double>(0, 1) / norm;
  axisAngle(0, 2) = rotationVector1x3.at<double>(0, 2) / norm;
  axisAngle(0, 3) = norm;

  return axisAngle;
}


//-----------------------------------------------------------------------------
cv::Mat RodriguesToEulerAngles(const cv::Mat& rotationVector1x3)
{
  cv::Mat eulerAngles = cvCreateMat(3,1,CV_64FC1);

  // This is not a straight forward conversion,
  // let's go Rodrigues->AxisAngle->Quaternion->Euler Angles
  cv::Matx14d axisAngle = niftk::RodriguesToAxisAngle(rotationVector1x3);

  double halfTheta = axisAngle (0,3) / 2;
  double q_i = axisAngle(0,0) * sin ( halfTheta );
  double q_j = axisAngle(0,1) * sin ( halfTheta );
  double q_k = axisAngle(0,2) * sin ( halfTheta );
  double q_r = cos ( halfTheta );

  // Roll
  eulerAngles.at<double>(0,0) = atan2 ( 2 * ( q_r * q_i + q_j * q_k ) ,
        1 - 2 * ( q_i * q_i + q_j * q_j ));
  // Pitch
  eulerAngles.at<double>(1,0) = asin ( 2 * ( q_r * q_j - q_k * q_i ) );

  // Yaw
  eulerAngles.at<double>(2,0) = atan2 ( 2 * ( q_r * q_k + q_i * q_j ) ,
        1 - 2 * ( q_j * q_j + q_k * q_k ));
  return eulerAngles;
}


//-----------------------------------------------------------------------------
cv::Mat AxisAngleToRodrigues(const cv::Matx14d& axisAngle)
{
  cv::Mat rodrigues = cvCreateMat(1, 3, CV_64FC1);
  rodrigues.at<double>(0, 0) = axisAngle(0, 0) * axisAngle(0, 3);
  rodrigues.at<double>(0, 1) = axisAngle(0, 1) * axisAngle(0, 3);
  rodrigues.at<double>(0, 2) = axisAngle(0, 2) * axisAngle(0, 3);
  return rodrigues;
}


//-----------------------------------------------------------------------------
std::vector<cv::Matx44d> MatrixListToVector(
    const std::list<cv::Matx44d>& list,
    const unsigned int& maximum
    )
{
  std::vector<cv::Matx44d> result;
  std::list<cv::Matx44d>::const_iterator iter;
  for (iter = list.begin();
       iter != list.end() && result.size() < maximum;
       ++iter
       )
  {
    result.push_back(*iter);
  }
  return result;
}


//-----------------------------------------------------------------------------
cv::Matx44d AverageMatricesUsingEigenValues(const std::list<cv::Matx44d >& matrices)
{
  if (matrices.empty())
  {
    niftkNiftyCalThrow() << "Empty matrices provided.";
  }

  double sizeAsDivisor = 1.0/static_cast<double>(matrices.size());
  cv::Matx33d tmp = cv::Matx33d::zeros();
  cv::Matx33d sizeAs3x3;
  cv::Matx31d tmpTranslation = cv::Matx31d::zeros();
  cv::Matx31d sizeAs3x1;

  std::list<cv::Matx44d >::const_iterator iter;
  for (iter = matrices.begin();
       iter != matrices.end();
       ++iter
       )
  {
    for ( int r = 0; r < 3; r++ )
    {
      for ( int c = 0; c < 3; c++ )
      {
        tmp(r, c) += (*iter)(r,c);
        sizeAs3x3(r,c) = sizeAsDivisor;
      }
      tmpTranslation(r, 0) += (*iter)(r, 3);
      sizeAs3x1(r, 0) = sizeAsDivisor;
    }
  }
  tmp = tmp.mul(sizeAs3x3);
  tmpTranslation = tmpTranslation.mul(sizeAs3x1);
  cv::Matx33d rtr = tmp.t() * tmp;

  cv::Matx33d eigenvectors;
  cv::Matx31d eigenvalues;
  cv::eigen(rtr , eigenvalues, eigenvectors);

  cv::Matx33d rootedEigenValues;

  for ( int r = 0; r < 3; r ++ )
  {
    for ( int c = 0; c < 3; c ++ )
    {
      if ( r == c )
      {
        rootedEigenValues(r, c) = sqrt(1.0/eigenvalues(r, 0));
      }
      else
      {
        rootedEigenValues(r, c) = 0.0;
      }
    }
  }

  cv::Matx44d returnMat = cv::Matx44d::eye();
  cv::Matx33d tmp2 = tmp * ( eigenvectors * rootedEigenValues * eigenvectors.t() );

  for ( int r = 0; r < 3; r ++ )
  {
    for ( int c = 0; c < 3; c ++ )
    {
      returnMat(r, c) = tmp2(r, c);
    }
    returnMat(r, 3) = tmpTranslation(r, 0);
  }
  return returnMat;
}


//-----------------------------------------------------------------------------
cv::Matx44d CalculateAverageModelToWorld(
    const cv::Matx44d&             handEyeMatrix,
    const std::list<cv::Matx44d >& handMatrices,
    const std::list<cv::Matx44d >& eyeMatrices
    )
{
  if (handMatrices.empty())
  {
    niftkNiftyCalThrow() << "Empty hand matrices provided.";
  }

  if (eyeMatrices.empty())
  {
    niftkNiftyCalThrow() << "Empty eye matrices provided.";
  }

  if (handMatrices.size() != eyeMatrices.size())
  {
    niftkNiftyCalThrow() << "Mismatching number of hand(" << handMatrices.size()
                         << ") and eye(" << eyeMatrices.size() << ") matrices provided.";
  }

  cv::Matx44d finalModelToWorld;
  std::list<cv::Matx44d > modelToWorlds;
  std::list<cv::Matx44d >::const_iterator handIter;
  std::list<cv::Matx44d >::const_iterator eyeIter;

  for (handIter = handMatrices.begin(),
       eyeIter = eyeMatrices.begin();
       handIter != handMatrices.end() && eyeIter != eyeMatrices.end();
       ++handIter,
       ++eyeIter
       )
  {
    cv::Matx44d handToTracker = *handIter;

    cv::Matx44d eyeToHand = handEyeMatrix.inv();
    cv::Matx44d modelToEye = *eyeIter;

    cv::Matx44d modelToWorld = handToTracker * eyeToHand * modelToEye;
    modelToWorlds.push_back(modelToWorld);
  }

  finalModelToWorld = AverageMatricesUsingEigenValues(modelToWorlds);
  return finalModelToWorld;
}


//-----------------------------------------------------------------------------
void InterpolateMaximumOfQuadraticSurface(
    const cv::Matx33d& matrix, cv::Point2d& outputPoint
    )
{
  outputPoint.x = 0;
  outputPoint.y = 0;

  cv::Point maxIndex;
  double maxValue = std::numeric_limits<double>::lowest();

  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      double value = matrix(r,c);

      if (value > maxValue)
      {
        maxIndex.x = c;
        maxIndex.y = r;
        maxValue = value;
      }
    }
  }
  // If the maximum of the given matrix is not the middle
  // pixel, give up, as interpolation will fail.
  if (maxIndex.x != 1 || maxIndex.y != 1)
  {
    return;
  }

  cv::Mat A = cvCreateMat ( 9, 6, CV_64FC1 );
  cv::Mat B = cvCreateMat ( 9, 1, CV_64FC1 );
  int rowCounter = 0;
  for (int y = -1; y <= 1; y++)
  {
    for (int x = -1; x <= 1; x++)
    {
      A.at<double>(rowCounter, 0) = x*x;
      A.at<double>(rowCounter, 1) = y*y;
      A.at<double>(rowCounter, 2) = x;
      A.at<double>(rowCounter, 3) = y;
      A.at<double>(rowCounter, 4) = x*y;
      A.at<double>(rowCounter, 5) = 1;
      B.at<double>(rowCounter, 0) = matrix(y+1, x+1);
      rowCounter++;
    }
  }
  cv::Mat invA = cvCreateMat ( 6, 9, CV_64FC1 );
  cv::invert(A, invA, cv::DECOMP_SVD);

  cv::Mat X = invA * B;

  double a = X.at<double>(0, 0);
  double b = X.at<double>(1, 0);
  double c = X.at<double>(2, 0);
  double d = X.at<double>(3, 0);
  double e = X.at<double>(4, 0);
//  double f = X.at<double>(5, 0); - not needed in 2nd deriv.

  double dxx = -1*(2*b*c - d*e)/(4*a*b - e*e);
  double dyy = -1*(2*a*d - c*e)/(4*a*b - e*e);

  // If offset > 1, interpolation must be a poor fit.
  if (fabs(dxx) > 1 || fabs(dyy) > 1)
  {
    return;
  }

  outputPoint.x = dxx;
  outputPoint.y = dyy;
}

} // end namespace
