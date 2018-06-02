/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereo3DCostFunction_h
#define niftkNonLinearStereo3DCostFunction_h

#include "niftkNonLinearStereoCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereo3DCostFunction
* \brief Base class for non-linear cost functions used to optimise 3D reconstruction error.
*/
class NonLinearStereo3DCostFunction : public NonLinearStereoCostFunction
{

public:

  typedef NonLinearStereo3DCostFunction Self;
  typedef NonLinearStereoCostFunction   Superclass;
  typedef itk::SmartPointer<Self>       Pointer;
  typedef itk::SmartPointer<const Self> ConstPointer;

  typedef Superclass::ParametersType    ParametersType;
  typedef Superclass::DerivativeType    DerivativeType;
  typedef Superclass::MeasureType       MeasureType;

  virtual unsigned int GetNumberOfValues(void) const ITK_OVERRIDE;

  void SetExtrinsics(std::vector<cv::Mat>* const rvecsLeft,
                     std::vector<cv::Mat>* const tvecsLeft,
                     cv::Mat* const leftToRightRotationMatrix,
                     cv::Mat* const leftToRightTranslationVector
                     );

protected:

  NonLinearStereo3DCostFunction();
  virtual ~NonLinearStereo3DCostFunction();

  NonLinearStereo3DCostFunction(const NonLinearStereo3DCostFunction&); // Purposefully not implemented.
  NonLinearStereo3DCostFunction& operator=(const NonLinearStereo3DCostFunction&); // Purposefully not implemented.

  std::vector<cv::Mat> *m_RvecsLeft;
  std::vector<cv::Mat> *m_TvecsLeft;
  cv::Mat              *m_LeftToRightRotationMatrix;
  cv::Mat              *m_LeftToRightTranslationVector;

};

} // end namespace

#endif
