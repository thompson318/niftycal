/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoCalibrationOptimiser_h
#define niftkNonLinearStereoCalibrationOptimiser_h

#include "niftkWin32ExportHeader.h"
#include <itkObject.h>
#include <itkObjectFactory.h>
#include <niftkNonLinearStereoCalibrationCostFunction.h>

namespace niftk
{

/**
* \class NonLinearStereoCalibrationOptimiser
* \brief Optimises standard OpenCV camera model by reconstructing the actual model position.
*
* \see niftk::NonLinearStereoCalibrationCostFunction
*/
class NIFTYCAL_WINEXPORT NonLinearStereoCalibrationOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoCalibrationOptimiser Self;
  typedef  itk::Object                         Superclass;
  typedef  itk::SmartPointer<Self>             Pointer;
  itkNewMacro(Self);

  void SetModel(Model3D* const model);
  void SetPoints(std::list<PointSet>* const points);
  void SetRightHandPoints(std::list<PointSet>* const points);

  double Optimise(cv::Mat& leftIntrinsic,
                  cv::Mat& leftDistortion,
                  cv::Mat& rightIntrinsic,
                  cv::Mat& rightDistortion,
                  cv::Mat& leftToRightRotationMatrix,
                  cv::Mat& leftToRightTranslationVector
                 );

protected:

  NonLinearStereoCalibrationOptimiser();
  virtual ~NonLinearStereoCalibrationOptimiser();

  NonLinearStereoCalibrationOptimiser(const NonLinearStereoCalibrationOptimiser&);
  NonLinearStereoCalibrationOptimiser& operator=(const NonLinearStereoCalibrationOptimiser&);

private:
  niftk::NonLinearStereoCalibrationCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
