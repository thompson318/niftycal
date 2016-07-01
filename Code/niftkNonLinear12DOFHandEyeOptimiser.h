/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinear12DOFHandEyeOptimiser_h
#define niftkNonLinear12DOFHandEyeOptimiser_h

#include "niftkWin32ExportHeader.h"
#include <itkObject.h>
#include <itkObjectFactory.h>
#include <niftkNonLinear12DOFHandEyeCostFunction.h>

namespace niftk
{

/**
* \class NonLinear12DOFHandEyeOptimiser
* \brief Optimises only hand-eye and model-to-world.
*/
class NIFTYCAL_WINEXPORT NonLinear12DOFHandEyeOptimiser : public itk::Object
{

public:

  typedef  NonLinear12DOFHandEyeOptimiser Self;
  typedef  itk::Object                    Superclass;
  typedef  itk::SmartPointer<Self>        Pointer;
  itkNewMacro(Self);

  void SetModel(Model3D* const model);
  void SetPoints(std::list<PointSet>* const points);
  void SetHandMatrices(std::list<cv::Matx44d>* const matrices);
  void SetIntrinsic(cv::Mat* const intrinsic);
  void SetDistortion(cv::Mat* const distortion);
  double Optimise(cv::Matx44d& modelToWorld, cv::Matx44d& handEye);

protected:

  NonLinear12DOFHandEyeOptimiser();
  virtual ~NonLinear12DOFHandEyeOptimiser();

  NonLinear12DOFHandEyeOptimiser(const NonLinear12DOFHandEyeOptimiser&);
  NonLinear12DOFHandEyeOptimiser& operator=(const NonLinear12DOFHandEyeOptimiser&);

private:
  niftk::NonLinear12DOFHandEyeCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif