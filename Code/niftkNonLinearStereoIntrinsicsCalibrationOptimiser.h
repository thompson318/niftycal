/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoIntrinsicsCalibrationOptimiser_h
#define niftkNonLinearStereoIntrinsicsCalibrationOptimiser_h

#include "niftkWin32ExportHeader.h"
#include <itkObject.h>
#include <itkObjectFactory.h>
#include <niftkNonLinearStereoIntrinsicsCalibrationCostFunction.h>

namespace niftk
{

/**
* \class NonLinearStereoIntrinsicsCalibrationOptimiser
* \brief Optimises standard OpenCV camera model by reconstructing the actual
* model (e.g. chessboard) positions, optimising only the intrinsic parameters.
*
* \see niftk::NonLinearStereoIntrinsicsCalibrationCostFunction
*/
class NIFTYCAL_WINEXPORT NonLinearStereoIntrinsicsCalibrationOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoIntrinsicsCalibrationOptimiser Self;
  typedef  itk::Object                                   Superclass;
  typedef  itk::SmartPointer<Self>                       Pointer;
  itkNewMacro(Self);

  void SetModelAndPoints(Model3D* const model,
                         std::list<PointSet>* const leftPoints,
                         std::list<PointSet>* const rightPoints
                         );

  void SetExtrinsics(std::vector<cv::Mat>* const rvecsLeft,
                     std::vector<cv::Mat>* const tvecsLeft,
                     cv::Mat* const leftToRightRotationMatrix,
                     cv::Mat* const leftToRightTranslationVector
                     );

  /**
  * \brief Optimises intrinsic parameters, and returns the 3D RMS reconstruction error.
  *
  * Note: You probably need a very good calibration before calling this.
  */
  double Optimise(cv::Mat& leftIntrinsic,
                  cv::Mat& leftDistortion,
                  cv::Mat& rightIntrinsic,
                  cv::Mat& rightDistortion
                 );

protected:

  NonLinearStereoIntrinsicsCalibrationOptimiser();
  virtual ~NonLinearStereoIntrinsicsCalibrationOptimiser();

  NonLinearStereoIntrinsicsCalibrationOptimiser(const NonLinearStereoIntrinsicsCalibrationOptimiser&);
  NonLinearStereoIntrinsicsCalibrationOptimiser& operator=(const NonLinearStereoIntrinsicsCalibrationOptimiser&);

private:

  niftk::NonLinearStereoIntrinsicsCalibrationCostFunction::Pointer m_CostFunction;

};

} // end namespace

#endif
