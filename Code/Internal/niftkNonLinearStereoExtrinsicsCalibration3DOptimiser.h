/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoExtrinsicsCalibration3DOptimiser_h
#define niftkNonLinearStereoExtrinsicsCalibration3DOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearStereoExtrinsicsCalibration3DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoExtrinsicsCalibration3DOptimiser
* \brief Optimises standard OpenCV camera model by reconstructing the actual model position,
* optimising extrinsic and stereo parameters.
*
* \see niftk::NonLinearStereoExtrinsicsCalibration3DCostFunction
*/
class NonLinearStereoExtrinsicsCalibration3DOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoExtrinsicsCalibration3DOptimiser Self;
  typedef  itk::Object                                     Superclass;
  typedef  itk::SmartPointer<Self>                         Pointer;
  itkNewMacro(Self);

  void SetModelAndPoints(const Model3D* const model,
                         const std::list<PointSet>* const leftPoints,
                         const std::list<PointSet>* const rightPoints
                        );

  void SetDistortionParameters(cv::Mat* const leftDistortion,
                               cv::Mat* const rightDistortion
                              );

  void SetIntrinsics(cv::Mat* const leftIntrinsic,
                     cv::Mat* const rightIntrinsic
                    );

  void SetOptimiseCameraExtrinsics(const bool& optimise);
  void SetOptimiseL2R(const bool& optimise);

  /**
  * \brief Optimises extrinsic parameters, and returns the 3D RMS reconstruction error.
  *
  * Note: You probably need a very good calibration before calling this.
  */
  double Optimise(std::vector<cv::Mat>& rvecsLeft,
                  std::vector<cv::Mat>& tvecsLeft,
                  cv::Mat& leftToRightRotationMatrix,
                  cv::Mat& leftToRightTranslationVector
                 );

protected:

  NonLinearStereoExtrinsicsCalibration3DOptimiser();
  virtual ~NonLinearStereoExtrinsicsCalibration3DOptimiser();

  NonLinearStereoExtrinsicsCalibration3DOptimiser(const NonLinearStereoExtrinsicsCalibration3DOptimiser&);
  NonLinearStereoExtrinsicsCalibration3DOptimiser& operator=(const NonLinearStereoExtrinsicsCalibration3DOptimiser&);

private:

  niftk::NonLinearStereoExtrinsicsCalibration3DCostFunction::Pointer m_CostFunction;

};

} // end namespace

#endif
