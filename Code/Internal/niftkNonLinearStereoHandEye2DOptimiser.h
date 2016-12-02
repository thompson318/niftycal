/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoHandEye2DOptimiser_h
#define niftkNonLinearStereoHandEye2DOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearStereoHandEye2DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoHandEye2DOptimiser
* \brief Optimises camera stereo extrinsic, hand-eye and model-to-world,
* as an extension of <a href="http://dx.doi.org/10.1002/rcs.1478">Malti 2013</a>.
*/
class NonLinearStereoHandEye2DOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoHandEye2DOptimiser Self;
  typedef  itk::Object                       Superclass;
  typedef  itk::SmartPointer<Self>           Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points);
  void SetRightHandPoints(const std::list<PointSet>* const points);
  void SetHandMatrices(const std::list<cv::Matx44d>* const matrices);
  void SetLeftIntrinsic(const cv::Mat* const intrinsic);
  void SetLeftDistortion(const cv::Mat* const distortion);
  void SetRightIntrinsic(const cv::Mat* const intrinsic);
  void SetRightDistortion(const cv::Mat* const distortion);

  double Optimise(cv::Matx44d& modelToWorld,
                  cv::Matx44d& handEye,
                  cv::Matx44d& stereoExtrinsics
                 );

protected:

  NonLinearStereoHandEye2DOptimiser();
  virtual ~NonLinearStereoHandEye2DOptimiser();

  NonLinearStereoHandEye2DOptimiser(const NonLinearStereoHandEye2DOptimiser&);
  NonLinearStereoHandEye2DOptimiser& operator=(const NonLinearStereoHandEye2DOptimiser&);

private:
  niftk::NonLinearStereoHandEye2DCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
