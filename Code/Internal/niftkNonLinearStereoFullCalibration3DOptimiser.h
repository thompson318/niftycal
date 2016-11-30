/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoFullCalibration3DOptimiser_h
#define niftkNonLinearStereoFullCalibration3DOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearStereoFullCalibration3DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoFullCalibration3DOptimiser
* \brief Optimises camera model by reconstructing the actual model position,
* optimising intrinsic, extrinsic and stereo parameters.
*
* \see niftk::NonLinearStereoFullCalibration3DCostFunction
*/
class NonLinearStereoFullCalibration3DOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoFullCalibration3DOptimiser Self;
  typedef  itk::Object                               Superclass;
  typedef  itk::SmartPointer<Self>                   Pointer;
  itkNewMacro(Self);

  void SetModelAndPoints(const Model3D* const model,
                         const std::list<PointSet>* const leftPoints,
                         const std::list<PointSet>* const rightPoints
                        );

  /**
  * \brief Optimises all parameters, and returns the 3D RMS reconstruction error.
  *
  * Note: You probably need a very good calibration before calling this.
  */
  double Optimise(cv::Mat& leftIntrinsic,
                  cv::Mat& leftDistortion,
                  cv::Mat& rightIntrinsic,
                  cv::Mat& rightDistortion,
                  std::vector<cv::Mat>& rvecsLeft,
                  std::vector<cv::Mat>& tvecsLeft,
                  cv::Mat& leftToRightRotationMatrix,
                  cv::Mat& leftToRightTranslationVector
                 );

protected:

  NonLinearStereoFullCalibration3DOptimiser();
  virtual ~NonLinearStereoFullCalibration3DOptimiser();

  NonLinearStereoFullCalibration3DOptimiser(const NonLinearStereoFullCalibration3DOptimiser&);
  NonLinearStereoFullCalibration3DOptimiser& operator=(const NonLinearStereoFullCalibration3DOptimiser&);

private:

  niftk::NonLinearStereoFullCalibration3DCostFunction::Pointer m_CostFunction;

};

} // end namespace

#endif
