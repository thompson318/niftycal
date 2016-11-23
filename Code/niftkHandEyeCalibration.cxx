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
#include <Internal/niftkNonLinearStereoHandEyeOptimiser.h>
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

#ifdef NIFTYCAL_WITH_ITK

//-----------------------------------------------------------------------------
cv::Matx44d CalculateHandEyeUsingMaltisMethod(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    listOfPointSets,
    const std::list<cv::Matx44d>& handMatrices,
    const std::list<cv::Matx44d>& eyeMatrices,
    cv::Mat&                      intrinsic,
    cv::Mat&                      distortion,
    double&                       residual
    )
{
  cv::Matx21d residuals;
  cv::Matx44d initialHandEye = niftk::CalculateHandEyeUsingTsaisMethod(handMatrices, eyeMatrices, residuals);
  cv::Matx44d initialModelToWorld = niftk::CalculateAverageModelToWorld(initialHandEye, handMatrices, eyeMatrices);

  cv::Matx44d finalHandEye = initialHandEye;

  niftk::NonLinearMaltiHandEyeOptimiser::Pointer optimiser = niftk::NonLinearMaltiHandEyeOptimiser::New();
  optimiser->SetModel(&model3D);
  optimiser->SetPoints(&listOfPointSets);
  optimiser->SetHandMatrices(&handMatrices);

  residual = optimiser->Optimise(initialModelToWorld,
                                 finalHandEye,
                                 intrinsic,
                                 distortion
                                );

  return finalHandEye;
}


//-----------------------------------------------------------------------------
cv::Matx44d CalculateHandEyeByOptimisingAllExtrinsic(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    listOfPointSets,
    const std::list<cv::Matx44d>& handMatrices,
    const std::list<cv::Matx44d>& eyeMatrices,
    const cv::Mat&                intrinsic,
    const cv::Mat&                distortion,
    double&                       residual
    )
{
  cv::Matx21d residuals;
  cv::Matx44d initialHandEye = niftk::CalculateHandEyeUsingTsaisMethod(handMatrices, eyeMatrices, residuals);
  cv::Matx44d initialModelToWorld = niftk::CalculateAverageModelToWorld(initialHandEye, handMatrices, eyeMatrices);

  cv::Matx44d finalHandEye = initialHandEye;

  niftk::NonLinearNDOFHandEyeOptimiser::Pointer optimiser = niftk::NonLinearNDOFHandEyeOptimiser::New();
  optimiser->SetModel(&model3D);
  optimiser->SetPoints(&listOfPointSets);
  optimiser->SetIntrinsic(&intrinsic);
  optimiser->SetDistortion(&distortion);
  optimiser->SetHandMatrices(&handMatrices);

  residual = optimiser->Optimise(initialModelToWorld, finalHandEye);

  return finalHandEye;
}


//-----------------------------------------------------------------------------
cv::Matx44d CalculateHandEyeInStereoByOptimisingAllExtrinsic(
    const niftk::Model3D&         model3D,
    const std::list<PointSet>&    leftPointSets,
    const cv::Mat&                leftIntrinsic,
    const cv::Mat&                leftDistortion,
    const std::list<PointSet>&    rightPointSets,
    const cv::Mat&                rightIntrinsic,
    const cv::Mat&                rightDistortion,
    const std::list<cv::Matx44d>& handMatrices,
    const std::list<cv::Matx44d>& eyeMatrices,
    cv::Matx44d&                  stereoExtrinsics,
    double&                       residual
    )
{
  cv::Matx21d residuals;
  cv::Matx44d initialHandEye = niftk::CalculateHandEyeUsingTsaisMethod(handMatrices, eyeMatrices, residuals);
  cv::Matx44d initialModelToWorld = niftk::CalculateAverageModelToWorld(initialHandEye, handMatrices, eyeMatrices);

  cv::Matx44d finalHandEye = initialHandEye;

  niftk::NonLinearStereoHandEyeOptimiser::Pointer optimiser = niftk::NonLinearStereoHandEyeOptimiser::New();
  optimiser->SetModel(&model3D);
  optimiser->SetPoints(&leftPointSets);
  optimiser->SetRightHandPoints(&rightPointSets);
  optimiser->SetHandMatrices(&handMatrices);
  optimiser->SetLeftIntrinsic(&leftIntrinsic);
  optimiser->SetLeftDistortion(&leftDistortion);
  optimiser->SetRightIntrinsic(&rightIntrinsic);
  optimiser->SetRightDistortion(&rightDistortion);

  residual = optimiser->Optimise(initialModelToWorld, finalHandEye, stereoExtrinsics);

  return finalHandEye;
}

#endif // NIFTYCAL_WITH_ITK

} // end namespace
