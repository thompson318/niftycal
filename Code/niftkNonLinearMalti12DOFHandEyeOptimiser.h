/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMalti12DOFHandEyeOptimiser_h
#define niftkNonLinearMalti12DOFHandEyeOptimiser_h

#include "niftkWin32ExportHeader.h"
#include <itkObject.h>
#include <itkObjectFactory.h>
#include <niftkNonLinearMalti12DOFHandEyeCostFunction.h>

namespace niftk
{

/**
* \brief Optimises only hand-eye and model-to-world.
*
* \see niftk::NonLinearMalti12DOFHandEyeCostFunction
* \see niftk::NonLinearMaltiHandEyeCostFunction
* \see niftk::NonLinearMaltiHandEyeOptimiser
*/
class NIFTYCAL_WINEXPORT NonLinearMalti12DOFHandEyeOptimiser : public itk::Object
{

public:

  typedef  NonLinearMalti12DOFHandEyeOptimiser Self;
  typedef  itk::Object                         Superclass;
  typedef  itk::SmartPointer<Self>             Pointer;
  itkNewMacro(Self);

  void SetModel(Model3D* const model);
  void SetPoints(std::list<PointSet>* const points);
  void SetHandMatrices(std::list<cv::Matx44d>* const matrices);
  void SetIntrinsic(cv::Mat* const intrinsic);
  void SetDistortion(cv::Mat* const distortion);
  double Optimise(cv::Matx44d& modelToWorld, cv::Matx44d& handEye);

protected:

  NonLinearMalti12DOFHandEyeOptimiser();
  virtual ~NonLinearMalti12DOFHandEyeOptimiser();

  NonLinearMalti12DOFHandEyeOptimiser(const NonLinearMalti12DOFHandEyeOptimiser&);
  NonLinearMalti12DOFHandEyeOptimiser& operator=(const NonLinearMalti12DOFHandEyeOptimiser&);

private:
  niftk::NonLinearMalti12DOFHandEyeCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
