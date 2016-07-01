/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoHandEyeOptimiser_h
#define niftkNonLinearStereoHandEyeOptimiser_h

#include "niftkWin32ExportHeader.h"
#include <itkObject.h>
#include <itkObjectFactory.h>
#include <niftkNonLinearStereoHandEyeCostFunction.h>

namespace niftk
{

/**
* \class NonLinearStereoHandEyeOptimiser
* \brief Optimises camera stereo extrinsic, hand-eye and model-to-world,
* as an extension of <a href="http://dx.doi.org/10.1002/rcs.1478">Malti 2013</a>.
*/
class NIFTYCAL_WINEXPORT NonLinearStereoHandEyeOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoHandEyeOptimiser Self;
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

  NonLinearStereoHandEyeOptimiser();
  virtual ~NonLinearStereoHandEyeOptimiser();

  NonLinearStereoHandEyeOptimiser(const NonLinearStereoHandEyeOptimiser&);
  NonLinearStereoHandEyeOptimiser& operator=(const NonLinearStereoHandEyeOptimiser&);

private:
  niftk::NonLinearStereoHandEyeCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
