/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoIntrinsicsCalibration3DOptimiser_h
#define niftkNonLinearStereoIntrinsicsCalibration3DOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearStereoIntrinsicsCalibration3DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoIntrinsicsCalibration3DOptimiser
* \brief Optimises standard OpenCV camera model by reconstructing the actual
* model (e.g. chessboard) positions, optimising only the intrinsic parameters.
*
* \see niftk::NonLinearStereoIntrinsicsCalibration3DCostFunction
*/
class NonLinearStereoIntrinsicsCalibration3DOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoIntrinsicsCalibration3DOptimiser Self;
  typedef  itk::Object                                     Superclass;
  typedef  itk::SmartPointer<Self>                         Pointer;
  itkNewMacro(Self);

  void SetModelAndPoints(const Model3D* const model,
                         const std::list<PointSet>* const leftPoints,
                         const std::list<PointSet>* const rightPoints
                        );

  void SetExtrinsics(std::vector<cv::Mat>* const rvecsLeft,
                     std::vector<cv::Mat>* const tvecsLeft,
                     cv::Mat* const leftToRightRotationMatrix,
                     cv::Mat* const leftToRightTranslationVector
                     );

  void SetDistortionParameters(cv::Mat* const leftDistortion,
                               cv::Mat* const rightDistortion
                               );

  /**
  * \brief Optimises intrinsic parameters, and returns the 3D RMS reconstruction error.
  *
  * Note: You probably need a very good calibration before calling this.
  */
  double Optimise(cv::Mat& leftIntrinsic,
                  cv::Mat& rightIntrinsic
                 );

protected:

  NonLinearStereoIntrinsicsCalibration3DOptimiser();
  virtual ~NonLinearStereoIntrinsicsCalibration3DOptimiser();

  NonLinearStereoIntrinsicsCalibration3DOptimiser(const NonLinearStereoIntrinsicsCalibration3DOptimiser&);
  NonLinearStereoIntrinsicsCalibration3DOptimiser& operator=(const NonLinearStereoIntrinsicsCalibration3DOptimiser&);

private:

  niftk::NonLinearStereoIntrinsicsCalibration3DCostFunction::Pointer m_CostFunction;

};

} // end namespace

#endif
