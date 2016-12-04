/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoHandEye3DOptimiser_h
#define niftkNonLinearStereoHandEye3DOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearStereoHandEye3DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoHandEye3DOptimiser
* \brief Optimises camera stereo extrinsic, hand-eye and model-to-world.
*/
class NonLinearStereoHandEye3DOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoHandEye3DOptimiser Self;
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

  NonLinearStereoHandEye3DOptimiser();
  virtual ~NonLinearStereoHandEye3DOptimiser();

  NonLinearStereoHandEye3DOptimiser(const NonLinearStereoHandEye3DOptimiser&);
  NonLinearStereoHandEye3DOptimiser& operator=(const NonLinearStereoHandEye3DOptimiser&);

private:
  niftk::NonLinearStereoHandEye3DCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
