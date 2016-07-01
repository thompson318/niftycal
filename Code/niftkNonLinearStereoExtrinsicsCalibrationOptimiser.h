/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoExtrinsicsCalibrationOptimiser_h
#define niftkNonLinearStereoExtrinsicsCalibrationOptimiser_h

#include "niftkWin32ExportHeader.h"
#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearStereoCalibrationOptimiser.h"
#include "niftkNonLinearStereoExtrinsicsCalibrationCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoExtrinsicsCalibrationOptimiser
* \brief Optimises standard OpenCV camera model by reconstructing the actual model position,
* optimising extrinsic and stereo parameters.
*
* \see niftk::NonLinearStereoExtrinsicsCalibrationCostFunction
*/
class NIFTYCAL_WINEXPORT NonLinearStereoExtrinsicsCalibrationOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoExtrinsicsCalibrationOptimiser Self;
  typedef  itk::Object                                   Superclass;
  typedef  itk::SmartPointer<Self>                       Pointer;
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

  NonLinearStereoExtrinsicsCalibrationOptimiser();
  virtual ~NonLinearStereoExtrinsicsCalibrationOptimiser();

  NonLinearStereoExtrinsicsCalibrationOptimiser(const NonLinearStereoExtrinsicsCalibrationOptimiser&);
  NonLinearStereoExtrinsicsCalibrationOptimiser& operator=(const NonLinearStereoExtrinsicsCalibrationOptimiser&);

private:

  niftk::NonLinearStereoExtrinsicsCalibrationCostFunction::Pointer m_CostFunction;

};

} // end namespace

#endif
