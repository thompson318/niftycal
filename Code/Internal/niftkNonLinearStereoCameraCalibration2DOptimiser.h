/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoCameraCalibration2DOptimiser_h
#define niftkNonLinearStereoCameraCalibration2DOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearStereoCameraCalibration2DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoCameraCalibration2DOptimiser
* \brief Optimises camera stereo intrinsic and extrinsic parameters.
*/
class NonLinearStereoCameraCalibration2DOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoCameraCalibration2DOptimiser Self;
  typedef  itk::Object                                 Superclass;
  typedef  itk::SmartPointer<Self>                     Pointer;
  itkNewMacro(Self);

  void SetModelAndPoints(const Model3D* const model,
                         const std::list<PointSet>* const leftPoints,
                         const std::list<PointSet>* const rightPoints
                        );

  /**
  * \brief Optimises all parameters, and returns the 2D RMS projection error.
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

  NonLinearStereoCameraCalibration2DOptimiser();
  virtual ~NonLinearStereoCameraCalibration2DOptimiser();

  NonLinearStereoCameraCalibration2DOptimiser(const NonLinearStereoCameraCalibration2DOptimiser&);
  NonLinearStereoCameraCalibration2DOptimiser& operator=(const NonLinearStereoCameraCalibration2DOptimiser&);

private:
  niftk::NonLinearStereoCameraCalibration2DCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
