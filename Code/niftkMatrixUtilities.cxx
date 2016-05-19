/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIOUtilities.h"
#include "niftkNiftyCalExceptionMacro.h"
#include <set>

namespace niftk {

//-----------------------------------------------------------------------------
cv::Matx44d RotationAndTranslationToMatrix(const cv::Mat& rotationMatrix,
                                           const cv::Mat& translationVector)
{
  if (rotationMatrix.rows != 3 || rotationMatrix.cols != 3)
  {
    niftkNiftyCalThrow() << "Invalid rotation matrix size.";
  }

  cv::Matx44d mat = cv::Matx44d::eye();

  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      mat(r,c) = rotationMatrix.at<double>(r,c);
    }
    mat(r,3) = translationVector.at<double>(0, r);
  }
  return mat;
}


//-----------------------------------------------------------------------------
cv::Matx44d RodriguesToMatrix(const cv::Mat& rotationVector,
                              const cv::Mat& translationVector)
{
  cv::Mat rotationMatrix;
  cv::Rodrigues(rotationVector, rotationMatrix);

  return RotationAndTranslationToMatrix(rotationMatrix, translationVector);
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
/**
* \brief Returns theta and P_r from equation 9 in Tsai's 1989 hand-eye paper.
*/
void MatrixToThetaAndPr(const cv::Matx44d& mat,
                        cv::Matx31d &axis,
                        double& angle
                       )
{
  cv::Mat rotationVector;
  cv::Mat translationVector;
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
  double maximumAngle = 0;
  unsigned int maximumAngleIndex = 0;
  cv::Matx31d notNeeded;

  alreadyChosen.insert(0);
  for (unsigned int i = 0; i < matrices.size(); i++)
  {
    maximumAngle = 0;
    for (unsigned int j = 1; j < matrices.size(); j++)
    {
      if (alreadyChosen.find(j) == alreadyChosen.end())
      {
        cv::Matx44d movementMatrix = matrices[j] * matrices[i].inv();
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
    std::cout << "Inserted:" << maximumAngleIndex << std::endl;
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

  finalHandEye = AverageMatricesUsingEigenValues(handEyes);
  return finalHandEye;
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
cv::Matx44d CalculateHandEyeUsingTsaisMethod(
    const std::list<cv::Matx44d >& handMatrices,
    const std::list<cv::Matx44d >& eyeMatrices,
    double& residualRotation,
    double& residualTranslation
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
  residualRotation = sqrt(errorRotationTransMult.at<double>(0,0)/static_cast<double>((numberOfViews-1)));

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
  double normPcgSquared = cv::norm(pcg);
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
  residualTranslation = sqrt(errorTranslationTransMult.at<double>(0,0)/(numberOfViews-1));

  cv::Matx44d eyeHand = cv::Matx44d::eye();
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      eyeHand(r, c) = rcg.at<double>(r, c);
    }
    eyeHand(r, 3) = tcg.at<double>(r, 0);
  }

  return eyeHand.inv();
}

} // end namespace
