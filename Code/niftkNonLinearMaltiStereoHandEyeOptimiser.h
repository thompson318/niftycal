/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMaltiStereoHandEyeOptimiser_h
#define niftkNonLinearMaltiStereoHandEyeOptimiser_h

#include "niftkWin32ExportHeader.h"
#include <itkObject.h>
#include <itkObjectFactory.h>
#include <niftkNonLinearMaltiStereoHandEyeCostFunction.h>

namespace niftk
{

/**
* \brief Optimises camera stereo extrinsic, hand-eye and model-to-world,
* as an extension of Malti 2013 http://dx.doi.org/10.1002/rcs.1478.
*
* \see niftk::NonLinearMaltiStereoHandEyeCostFunction
*/
class NIFTYCAL_WINEXPORT NonLinearMaltiStereoHandEyeOptimiser : public itk::Object
{

public:

  typedef  NonLinearMaltiStereoHandEyeOptimiser Self;
  typedef  itk::Object                        Superclass;
  typedef  itk::SmartPointer<Self>            Pointer;
  itkNewMacro(Self);

  void SetModel(Model3D* const model);
  void SetPoints(std::list<PointSet>* const points);
  void SetRightHandPoints(std::list<PointSet>* const points);
  void SetHandMatrices(std::list<cv::Matx44d>* const matrices);
  void SetLeftIntrinsic(cv::Mat* const intrinsic);
  void SetLeftDistortion(cv::Mat* const distortion);
  void SetRightIntrinsic(cv::Mat* const intrinsic);
  void SetRightDistortion(cv::Mat* const distortion);

  double Optimise(cv::Matx44d& modelToWorld,
                  cv::Matx44d& handEye,
                  cv::Matx44d& stereoExtrinsics
                 );

protected:

  NonLinearMaltiStereoHandEyeOptimiser();
  virtual ~NonLinearMaltiStereoHandEyeOptimiser();

  NonLinearMaltiStereoHandEyeOptimiser(const NonLinearMaltiStereoHandEyeOptimiser&);
  NonLinearMaltiStereoHandEyeOptimiser& operator=(const NonLinearMaltiStereoHandEyeOptimiser&);

private:
  niftk::NonLinearMaltiStereoHandEyeCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
