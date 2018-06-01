/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkHandEyeCalibration.h"
#include "niftkNiftyCalExceptionMacro.h"

#ifdef NIFTYCAL_WITH_ITK
#include <Internal/niftkNonLinearMaltiHandEyeOptimiser.h>
#include <Internal/niftkNonLinearNDOFHandEyeOptimiser.h>
#include <Internal/niftkNonLinearStereoHandEye2DOptimiser.h>
#include <Internal/niftkNonLinearStereoHandEye3DOptimiser.h>
#endif

namespace niftk {

//-----------------------------------------------------------------------------
/**
* \brief Returns theta and P_r from equation 9 in Tsai's 1989 hand-eye paper.
*/
void MatrixToThetaAndPr(const cv::Matx44d& mat,
                        cv::Matx31d &axis,
                        double& angle
                       )
{
  cv::Mat rotationVector = cvCreateMat ( 1, 3, CV_64FC1 );
  cv::Mat translationVector = cvCreateMat ( 1, 3, CV_64FC1 );
  niftk::MatrixToRodrigues(mat, rotationVector, translationVector);
  double norm = cv::norm(rotationVector);
  rotationVector /= norm;                // gives unit vector.
  rotationVector *= (2.0*sin(norm/2.0)); // see eqn. (9) in Tsai's 1989 hand-eye paper.

  angle = norm;
  axis(0, 0) = rotationVector.at<double>(0, 0);
  axis(1, 0) = rotationVector.at<double>(0, 1);
  axis(2, 0) = rotationVector.at<double>(0, 2);
}


//-----------------------------------------------------------------------------
/**
* \brief Returns a vector of matrices indexes, to maximumise interstation angle,
* as shown in Tsai's 1989 hand-eye paper, Figure 6.
* \param matrices must be an odd number of matrices.
*/
std::vector<unsigned int> ExtractMaximumDistanceIndexes(const std::vector<cv::Matx44d>& matrices)
{
  if (matrices.size() % 2 != 1)
  {
    niftkNiftyCalThrow() << "Must be an odd number of entries.";
  }

  std::vector<unsigned int> result;
  std::set<unsigned int> alreadyChosen;

  result.push_back(0);

  if (matrices.size() == 1)
  {
    return result;
  }


  double angle = 0;
  unsigned int maximumAngleIndex = 0;
  cv::Matx31d notNeeded;

  unsigned int previouslyChosenIndex = 0;
  alreadyChosen.insert(0);

  while(alreadyChosen.size() < matrices.size())
  {
    double maximumAngle = 0;
    for (unsigned int j = 0; j < matrices.size(); j++)
    {
      if (alreadyChosen.find(j) == alreadyChosen.end())
      {
        cv::Matx44d movementMatrix = matrices[j] * matrices[previouslyChosenIndex].inv();
        niftk::MatrixToThetaAndPr(movementMatrix, notNeeded, angle);
        if (angle > maximumAngle)
        {
          maximumAngleIndex = j;
          maximumAngle = angle;
        }
      }
    }
    result.push_back(maximumAngleIndex);
    alreadyChosen.insert(maximumAngleIndex);
    previouslyChosenIndex = maximumAngleIndex;
  }

  return result;
}


//-----------------------------------------------------------------------------
void CalculateHandEyeUsingPoint2Line(
    const cv::Mat&                  cameraMatrix,
    const cv::Point3d&              stylusOrigin,
    const std::vector<cv::Matx44d>& stylusTrackingMatrices,
    const std::vector<cv::Matx44d>& trackingMatrices,
    const std::vector<cv::Point2d>& undistortedPoints,
    const double&                   exitCondition,
    cv::Matx44d&                    handEye
    )
{
  if (stylusTrackingMatrices.size() != undistortedPoints.size())
  {
    niftkNiftyCalThrow() << "Unequal number of stylus matrices (" << stylusTrackingMatrices.size()
                         << ") and undistorted points (" << undistortedPoints.size()
                         << ")" << std::endl;
  }
  if (trackingMatrices.size() != undistortedPoints.size())
  {
    niftkNiftyCalThrow() << "Unequal number of tracking matrices (" << trackingMatrices.size()
                         << ") and undistorted points (" << undistortedPoints.size()
                         << ")" << std::endl;
  }
  std::vector<std::pair<cv::Point2d, cv::Point3d> > data;
  cv::Matx41d inputPoint;
  inputPoint(3, 0) = 1;
  cv::Matx41d transformedPoint;
  cv::Point3d point3D;

  for (int i = 0; i < undistortedPoints.size(); i++)
  {
    inputPoint(0, 0) = stylusOrigin.x;
    inputPoint(1, 0) = stylusOrigin.y;
    inputPoint(2, 0) = stylusOrigin.z;
    transformedPoint = (trackingMatrices[i].inv()) * stylusTrackingMatrices[i] * inputPoint;
    point3D.x = transformedPoint(0, 0);
    point3D.y = transformedPoint(1, 0);
    point3D.z = transformedPoint(2, 0);

    data.push_back(std::pair<cv::Point2d, cv::Point3d>(undistortedPoints[i], point3D));
  }
  CalculateHandEyeUsingPoint2Line(cameraMatrix, data, exitCondition, handEye);
}


//-----------------------------------------------------------------------------
void CalculateHandEyeUsingPoint2Line(
    const cv::Mat&                  cameraMatrix,
    const std::vector<cv::Matx44d>& trackingMatrices,
    const std::vector<cv::Point3d>& pointsInTrackerSpace,
    const std::vector<cv::Point2d>& undistortedPoints,
    const double&                   exitCondition,
    cv::Matx44d&                    handEye
    )
{
  if (pointsInTrackerSpace.size() != undistortedPoints.size())
  {
    niftkNiftyCalThrow() << "Unequal number of tracker points (" << pointsInTrackerSpace.size()
                         << ") and undistorted points (" << undistortedPoints.size()
                         << ")" << std::endl;
  }
  if (trackingMatrices.size() != undistortedPoints.size())
  {
    niftkNiftyCalThrow() << "Unequal number of tracking matrices (" << trackingMatrices.size()
                         << ") and undistorted points (" << undistortedPoints.size()
                         << ")" << std::endl;
  }
  std::vector<std::pair<cv::Point2d, cv::Point3d> > data;
  cv::Matx41d inputPoint;
  inputPoint(3, 0) = 1;
  cv::Matx41d transformedPoint;
  cv::Point3d point3D;

  for (int i = 0; i < undistortedPoints.size(); i++)
  {
    inputPoint(0, 0) = pointsInTrackerSpace[i].x;
    inputPoint(1, 0) = pointsInTrackerSpace[i].y;
    inputPoint(2, 0) = pointsInTrackerSpace[i].z;
    transformedPoint = (trackingMatrices[i].inv()) * inputPoint;
    point3D.x = transformedPoint(0, 0);
    point3D.y = transformedPoint(1, 0);
    point3D.z = transformedPoint(2, 0);

    data.push_back(std::pair<cv::Point2d, cv::Point3d>(undistortedPoints[i], point3D));
  }
  CalculateHandEyeUsingPoint2Line(cameraMatrix, data, exitCondition, handEye);
}


/**
 From Morgan et al. IPCAI 2017

 MATLAB code provided in paper:

% INPUTS: X : (3xn) 3D coordinates (tracker space).
%         Q : (2xn) 2D pixel coordinates (image space).
%         A : (3x3) camera Matrix.
%       tol : exit condition.
% OUTPUTS R : 3x3 rotation matrix
%         t : 3x1 translation vector.
%
% n = size(Q,2); e = ones(1,n); j=eye(n)-((e'*e)./n);
% Q = normc(inv(A)*[Q;e]);
% Y = Q;
% err = +Inf; E_old = 1000*ones(3,n);
% while err > tol
%   [U,~,V] = svd(Y*J*X');
%   R = U*[1 0 0; 0 1 0; 0 0 det (U*V')]*V';
%   t = mean(Y-R*X, 2);
%   Y = repmat(dot(R*X + t*e, Q), [3,1]) .* Q;
%   E = Y-R*X - t*e;
%   err = norm(E-E_old, ' fro '); E_old = E;
% end
*/
//-----------------------------------------------------------------------------
void CalculateHandEyeUsingPoint2Line(
    const cv::Mat&                                           cameraMatrix,
    const std::vector<std::pair<cv::Point2d, cv::Point3d> >& pairedPoints,
    const double&                                            tol,
    cv::Matx44d&                                             handEye
    )
{
  int n = pairedPoints.size();
  cv::Mat e = cv::Mat::ones( 1, n, CV_64FC1 );

  cv::Mat ePrime;
  cv::transpose(e, ePrime);

  cv::Mat J = cv::Mat::eye(n, n, CV_64FC1 ) - ((ePrime * e)/n);

  cv::Mat Q = cvCreateMat ( 2, n, CV_64FC1 );
  cv::Mat Qe = cvCreateMat ( 3, n, CV_64FC1 );
  cv::Mat X = cvCreateMat ( 3, n, CV_64FC1 );

  for (int i = 0; i < n; i++)
  {
    Q.at<double>(0, i)  = pairedPoints[i].first.x;
    Q.at<double>(1, i)  = pairedPoints[i].first.y;
    Qe.at<double>(0, i) = pairedPoints[i].first.x;
    Qe.at<double>(1, i) = pairedPoints[i].first.y;
    Qe.at<double>(2, i) = e.at<double>(0, i);
    X.at<double>(0, i)  = pairedPoints[i].second.x;
    X.at<double>(1, i)  = pairedPoints[i].second.y;
    X.at<double>(2, i)  = pairedPoints[i].second.z;
  }

  cv::Mat AInv = cameraMatrix.inv();
  cv::Mat lines = AInv * Qe;

  assert(lines.rows == Qe.rows);
  assert(lines.cols == Qe.cols);

  // Normalise columns
  for (int c = 0; c < lines.cols; c++)
  {
    double magnitude = 0;
    for (int r = 0; r < lines.rows; r++)
    {
      magnitude += (lines.at<double>(r, c) * lines.at<double>(r, c));
    }
    magnitude = sqrt(magnitude);

    for (int r = 0; r < lines.rows; r++)
    {
      lines.at<double>(r, c) = lines.at<double>(r, c) / magnitude;
    }
  }
  lines.copyTo(Q); // Assigns 3xn matrix to Q, so Q is now 3xn not 2xn

  // Set up X' for SVD part.
  cv::Mat XPrime;
  cv::transpose(X, XPrime);

  // Setup loop.
  cv::Mat Y;
  Q.copyTo(Y);

  double err = std::numeric_limits<double>::max();
  cv::Mat E_old = 1000*cv::Mat::ones(3, n, CV_64FC1);
  cv::Mat E;
  cv::Mat R = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat t = cv::Mat::zeros(3, 1, CV_64FC1);

  // Iterate to minimise error.
  while (err > tol)
  {
    cv::SVD svd(Y*J*XPrime);
    cv::Mat U = svd.u;
    cv::Mat VPrime = svd.vt;

    double det = cv::determinant(U*VPrime);

    cv::Mat detMatrix = cv::Mat::eye(3, 3, CV_64FC1);
    detMatrix.at<double>(2, 2) = det;

    R = U * detMatrix * VPrime;

    cv::Mat yMinusRotatedX = Y - R*X;

    // mean along row
    for (int r = 0; r < yMinusRotatedX.rows; r++)
    {
      double mean = 0;
      for (int c = 0; c < yMinusRotatedX.cols; c++)
      {
        mean += yMinusRotatedX.at<double>(r,c);
      }
      mean /= static_cast<double>(yMinusRotatedX.cols);
      t.at<double>(r, 0) = mean;
    }

    // Next section should be:
    // Y = repmat(dot(R*X+t*e,Q), [3,1]).* Q;
    cv::Mat rotatedXPlusTrans = R*X + t*e;
    cv::Mat dot = cv::Mat::zeros(1, n, CV_64FC1);

    assert(rotatedXPlusTrans.rows == Q.rows);
    assert(rotatedXPlusTrans.cols == Q.cols);

    // Dot product bit.
    for (int r = 0; r < Q.rows; r++)
    {
      for (int c = 0; c < Q.cols; c++)
      {
        dot.at<double>(0, c) += rotatedXPlusTrans.at<double>(r, c) * Q.at<double>(r,c);
      }
    }
    // Repmat and element-wise multiplication by Q.
    for (int r = 0; r < Y.rows; r++)
    {
      for (int c = 0; c < Y.cols; c++)
      {
        Y.at<double>(r, c) = dot.at<double>(0, c) * Q.at<double>(r,c);
      }
    }

    yMinusRotatedX = Y - R*X; // as y has been updated.
    E = yMinusRotatedX - t * e;

    cv::Mat residual = E-E_old;
    cv::Mat residualTransposed;
    cv::transpose(residual, residualTransposed);
    cv::Mat squaredResiduals = residualTransposed * residual;
    cv::Mat trace = squaredResiduals.diag(0);
    err = std::sqrt(cv::sum(trace)[0]); // Frobenius norm of E - E_old
    E.copyTo(E_old);
  }

  cv::Matx44d eyeHand = cv::Matx44d::eye();
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      eyeHand(r, c) = R.at<double>(r, c);
    }
    eyeHand(r, 3) = t.at<double>(r, 0);
  }

  handEye = eyeHand.inv();
}


//-----------------------------------------------------------------------------
cv::Matx44d CalculateHandEyeUsingTsaisMethod(
    const std::list<cv::Matx44d >& handMatrices,
    const std::list<cv::Matx44d >& eyeMatrices,
    cv::Matx21d&                   residual
    )
{
  if (handMatrices.size() < 3)
  {
    niftkNiftyCalThrow() << "Not enough hand matrices provided.";
  }

  if (eyeMatrices.size() < 3)
  {
    niftkNiftyCalThrow() << "Not enough eye matrices provided.";
  }

  if (handMatrices.size() != eyeMatrices.size())
  {
    niftkNiftyCalThrow() << "Mismatching number of hand(" << handMatrices.size()
                         << ") and eye(" << eyeMatrices.size() << ") matrices provided.";
  }

  unsigned int maximumSize = handMatrices.size();
  if (maximumSize % 2 == 0)
  {
    maximumSize--;
  }

  std::vector<cv::Matx44d> hands = MatrixListToVector(handMatrices, maximumSize);
  std::vector<cv::Matx44d> eyes = MatrixListToVector(eyeMatrices, maximumSize);
  std::vector<unsigned int> reorderedIndexes = ExtractMaximumDistanceIndexes(hands);

  int numberOfViews = hands.size(); // which may now be different size (1 less) than handMatrices.
  cv::Matx31d Pcij;
  cv::Matx31d Pgij;
  double ThetaCij;
  double ThetaGij;

  cv::Mat A = cvCreateMat ( 3 * (numberOfViews - 1), 3, CV_64FC1 );
  cv::Mat b = cvCreateMat ( 3 * (numberOfViews - 1), 1, CV_64FC1 );

  // Step 1: Filling A, b for least squares solution of pcgPrime.
  for (int i = 0; i < numberOfViews - 1; i++)
  {
    cv::Matx44d handMovement = hands[reorderedIndexes[i+1]].inv() * hands[reorderedIndexes[i]];
    cv::Matx44d eyeMovement = eyes[reorderedIndexes[i+1]] * eyes[reorderedIndexes[i]].inv();

    niftk::MatrixToThetaAndPr(handMovement, Pgij, ThetaGij);
    niftk::MatrixToThetaAndPr(eyeMovement, Pcij, ThetaCij);

    cv::Mat sum = cvCreateMat(3, 1, CV_64FC1);
    sum.at<double>(0, 0) = Pcij(0, 0) + Pgij(0, 0);
    sum.at<double>(1, 0) = Pcij(1, 0) + Pgij(1, 0);
    sum.at<double>(2, 0) = Pcij(2, 0) + Pgij(2, 0);

    cv::Mat diff = cvCreateMat(3, 1, CV_64FC1);
    diff.at<double>(0, 0) = Pcij(0, 0) - Pgij(0, 0);
    diff.at<double>(1, 0) = Pcij(1, 0) - Pgij(1, 0);
    diff.at<double>(2, 0) = Pcij(2, 0) - Pgij(2, 0);

    A.at<double>(i*3+0,0)=0.0;
    A.at<double>(i*3+0,1)=-(sum.at<double>(2,0));
    A.at<double>(i*3+0,2)=sum.at<double>(1,0);
    A.at<double>(i*3+1,0)=sum.at<double>(2,0);
    A.at<double>(i*3+1,1)=0.0;
    A.at<double>(i*3+1,2)=-(sum.at<double>(0,0));
    A.at<double>(i*3+2,0)=-(sum.at<double>(1,0));
    A.at<double>(i*3+2,1)=sum.at<double>(0,0);
    A.at<double>(i*3+2,2)=0.0;

    b.at<double>(i*3+0,0)=diff.at<double>(0,0);
    b.at<double>(i*3+1,0)=diff.at<double>(1,0);
    b.at<double>(i*3+2,0)=diff.at<double>(2,0);
  }

  cv::Mat pseudoInverse = cvCreateMat(3,3,CV_64FC1);
  cv::invert(A, pseudoInverse, CV_SVD);

  // Step 1. Here we have Pcg'
  cv::Mat pcgPrime = pseudoInverse * b;

  // Here calculating RMS error for rotation.
  cv::Mat errorRotation = A * pcgPrime - b;
  cv::Mat errorRotationTransMult = cvCreateMat(errorRotation.cols, errorRotation.cols, CV_64FC1);
  cv::mulTransposed (errorRotation, errorRotationTransMult, true);
  residual(0, 0) = sqrt(errorRotationTransMult.at<double>(0,0)/static_cast<double>((numberOfViews-1)));

  // Step 3. Compute Pcg.
  cv::Mat pcg = 2 * pcgPrime / (sqrt(1 + cv::norm(pcgPrime) * cv::norm(pcgPrime)));

  // Start, computing Rcg, using Pcg and Eqn. 10.
  cv::Mat id3 = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat pcgCrossProduct = cvCreateMat(3,3,CV_64FC1);
  pcgCrossProduct.at<double>(0,0)=0.0;
  pcgCrossProduct.at<double>(0,1)=-(pcg.at<double>(2,0));
  pcgCrossProduct.at<double>(0,2)=(pcg.at<double>(1,0));
  pcgCrossProduct.at<double>(1,0)=(pcg.at<double>(2,0));
  pcgCrossProduct.at<double>(1,1)=0.0;
  pcgCrossProduct.at<double>(1,2)=-(pcg.at<double>(0,0));
  pcgCrossProduct.at<double>(2,0)=-(pcg.at<double>(1,0));
  pcgCrossProduct.at<double>(2,1)=(pcg.at<double>(0,0));
  pcgCrossProduct.at<double>(2,2)=0.0;

  cv::Mat pcgMulTransposed = cvCreateMat(pcg.rows, pcg.rows, CV_64FC1);
  cv::mulTransposed (pcg, pcgMulTransposed, false);

  // Eqn. 10, giving us the rotation we are looking for.
  double normPcgSquared = cv::norm(pcg)*cv::norm(pcg);
  double alpha = sqrt(4 - normPcgSquared);
  cv::Mat rcg = ( 1 - (normPcgSquared/2.0) ) * id3
      + 0.5 * ( pcgMulTransposed + alpha*pcgCrossProduct);

  // Step 4, computing tcg.
  for ( int i = 0; i < numberOfViews - 1; i ++ )
  {
    cv::Matx44d handMovement = hands[reorderedIndexes[i+1]].inv() * hands[reorderedIndexes[i]];
    cv::Matx44d eyeMovement = eyes[reorderedIndexes[i+1]] * eyes[reorderedIndexes[i]].inv();

    niftk::MatrixToThetaAndPr(handMovement, Pgij, ThetaGij);
    niftk::MatrixToThetaAndPr(eyeMovement, Pcij, ThetaCij);

    A.at<double>(i*3+0,0)=handMovement(0,0) - 1.0;
    A.at<double>(i*3+0,1)=handMovement(0,1) - 0.0;
    A.at<double>(i*3+0,2)=handMovement(0,2) - 0.0;
    A.at<double>(i*3+1,0)=handMovement(1,0) - 0.0;
    A.at<double>(i*3+1,1)=handMovement(1,1) - 1.0;
    A.at<double>(i*3+1,2)=handMovement(1,2) - 0.0;
    A.at<double>(i*3+2,0)=handMovement(2,0) - 0.0;
    A.at<double>(i*3+2,1)=handMovement(2,1) - 0.0;
    A.at<double>(i*3+2,2)=handMovement(2,2) - 1.0;

    cv::Mat Tgij = cvCreateMat(3,1,CV_64FC1);
    cv::Mat Tcij = cvCreateMat(3,1,CV_64FC1);

    for ( int j = 0; j < 3; j ++ )
    {
      Tgij.at<double>(j, 0) = handMovement(j, 3);
      Tcij.at<double>(j, 0) = eyeMovement(j, 3);
    }
    cv::Mat dTransposed = rcg * Tcij - Tgij; // Note: requires rcg from above.
                                             //       so we can't merge the for loops.

    b.at<double>(i*3+0,0)=dTransposed.at<double>(0,0);
    b.at<double>(i*3+1,0)=dTransposed.at<double>(1,0);
    b.at<double>(i*3+2,0)=dTransposed.at<double>(2,0);
  }

  cv::invert(A, pseudoInverse, CV_SVD);
  cv::Mat tcg = pseudoInverse * b;

  cv::Mat errorTranslation = A * tcg - b;
  cv::Mat errorTranslationTransMult = cvCreateMat(errorTranslation.cols, errorTranslation.cols, CV_64FC1);
  cv::mulTransposed (errorTranslation, errorTranslationTransMult, true);
  residual(1, 0) = sqrt(errorTranslationTransMult.at<double>(0,0)/(numberOfViews-1));

  cv::Matx44d eyeHand = cv::Matx44d::eye();
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      eyeHand(r, c) = rcg.at<double>(r, c);
    }
    eyeHand(r, 3) = tcg.at<double>(r, 0);
  }

  cv::Matx44d handEye = eyeHand.inv(cv::DECOMP_SVD);
  handEye(3,0) = 0;
  handEye(3,1) = 0;
  handEye(3,2) = 0;
  handEye(3,3) = 1;
  return handEye;
}


//-----------------------------------------------------------------------------
cv::Matx44d CalculateHandEyeByDirectMatrixMultiplication(
    const cv::Matx44d&             modelToTrackerTransform,
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

  cv::Matx44d finalHandEye;
  std::list<cv::Matx44d > handEyes;
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

    cv::Matx44d trackerToModel = modelToTrackerTransform.inv();
    cv::Matx44d modelToEye = *eyeIter;

    cv::Matx44d handEye = modelToEye * trackerToModel * handToTracker;
    handEyes.push_back(handEye);
  }

  finalHandEye = niftk::AverageMatricesUsingEigenValues(handEyes);
  return finalHandEye;
}


//-----------------------------------------------------------------------------
void CalculateHandEyeUsingMaltisMethod(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    listOfPointSets,
    const std::list<cv::Matx44d>& handMatrices,
    cv::Mat&                      intrinsic,
    cv::Mat&                      distortion,
    cv::Matx44d&                  handEye,
    cv::Matx44d&                  modelToWorld,
    double&                       residual
    )
{
  residual = 0;

#ifdef NIFTYCAL_WITH_ITK

  niftk::NonLinearMaltiHandEyeOptimiser::Pointer optimiser = niftk::NonLinearMaltiHandEyeOptimiser::New();
  optimiser->SetModel(&model3D);
  optimiser->SetPoints(&listOfPointSets);
  optimiser->SetHandMatrices(&handMatrices);

  residual = optimiser->Optimise(modelToWorld,
                                 handEye,
                                 intrinsic,
                                 distortion
                                );
#endif
}


//-----------------------------------------------------------------------------
void CalculateHandEyeByOptimisingAllExtrinsic(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    listOfPointSets,
    const std::list<cv::Matx44d>& handMatrices,
    const cv::Mat&                intrinsic,
    const cv::Mat&                distortion,
    cv::Matx44d&                  handEye,
    cv::Matx44d&                  modelToWorld,
    double&                       residual
    )
{
  residual = 0;

#ifdef NIFTYCAL_WITH_ITK

  niftk::NonLinearNDOFHandEyeOptimiser::Pointer optimiser = niftk::NonLinearNDOFHandEyeOptimiser::New();
  optimiser->SetModel(&model3D);
  optimiser->SetPoints(&listOfPointSets);
  optimiser->SetIntrinsic(&intrinsic);
  optimiser->SetDistortion(&distortion);
  optimiser->SetHandMatrices(&handMatrices);

  residual = optimiser->Optimise(modelToWorld, handEye);

#endif
}


//-----------------------------------------------------------------------------
void CalculateHandEyeInStereoByOptimisingAllExtrinsic(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    leftPointSets,
    const cv::Mat&                leftIntrinsic,
    const cv::Mat&                leftDistortion,
    const std::list<PointSet>&    rightPointSets,
    const cv::Mat&                rightIntrinsic,
    const cv::Mat&                rightDistortion,
    const std::list<cv::Matx44d>& handMatrices,
    const bool&                   optimise3D,
    cv::Matx44d&                  handEye,
    cv::Matx44d&                  modelToWorld,
    cv::Matx44d&                  stereoExtrinsics,
    double&                       residual
    )
{
  residual = 0;

#ifdef NIFTYCAL_WITH_ITK

  niftk::NonLinearStereoHandEye2DOptimiser::Pointer optimiser2D = niftk::NonLinearStereoHandEye2DOptimiser::New();
  optimiser2D->SetModel(&model3D);
  optimiser2D->SetPoints(&leftPointSets);
  optimiser2D->SetRightHandPoints(&rightPointSets);
  optimiser2D->SetHandMatrices(&handMatrices);
  optimiser2D->SetLeftIntrinsic(&leftIntrinsic);
  optimiser2D->SetLeftDistortion(&leftDistortion);
  optimiser2D->SetRightIntrinsic(&rightIntrinsic);
  optimiser2D->SetRightDistortion(&rightDistortion);

  residual = optimiser2D->Optimise(modelToWorld, handEye, stereoExtrinsics);

  if (optimise3D)
  {
    niftk::NonLinearStereoHandEye3DOptimiser::Pointer optimiser3D = niftk::NonLinearStereoHandEye3DOptimiser::New();
    optimiser3D->SetModel(&model3D);
    optimiser3D->SetPoints(&leftPointSets);
    optimiser3D->SetRightHandPoints(&rightPointSets);
    optimiser3D->SetHandMatrices(&handMatrices);
    optimiser3D->SetLeftIntrinsic(&leftIntrinsic);
    optimiser3D->SetLeftDistortion(&leftDistortion);
    optimiser3D->SetRightIntrinsic(&rightIntrinsic);
    optimiser3D->SetRightDistortion(&rightDistortion);

    residual = optimiser3D->Optimise(modelToWorld, handEye, stereoExtrinsics);
  }

#endif
}

} // end namespace
