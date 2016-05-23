/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMaltiNDOFHandEyeOptimiser_h
#define niftkNonLinearMaltiNDOFHandEyeOptimiser_h

#include "niftkWin32ExportHeader.h"
#include <itkObject.h>
#include <itkObjectFactory.h>
#include <niftkNonLinearMaltiNDOFHandEyeCostFunction.h>

namespace niftk
{

/**
* \brief Optimises camera extrinsic, hand-eye and model-to-world.
*
* \see niftk::NonLinearMaltiNDOFHandEyeCostFunction
*/
class NIFTYCAL_WINEXPORT NonLinearMaltiNDOFHandEyeOptimiser : public itk::Object
{

public:

  typedef  NonLinearMaltiNDOFHandEyeOptimiser Self;
  typedef  itk::Object                        Superclass;
  typedef  itk::SmartPointer<Self>            Pointer;
  itkNewMacro(Self);

  void SetModel(Model3D* const model);
  void SetPoints(std::list<PointSet>* const points);
  void SetHandMatrices(std::list<cv::Matx44d>* const matrices);
  void SetIntrinsic(cv::Mat* const intrinsic);
  void SetDistortion(cv::Mat* const distortion);
  double Optimise(cv::Matx44d& modelToWorld,
                  cv::Matx44d& handEye,
                  std::list<cv::Matx44d>* const matrices
                 );

protected:

  NonLinearMaltiNDOFHandEyeOptimiser();
  virtual ~NonLinearMaltiNDOFHandEyeOptimiser();

  NonLinearMaltiNDOFHandEyeOptimiser(const NonLinearMaltiNDOFHandEyeOptimiser&);
  NonLinearMaltiNDOFHandEyeOptimiser& operator=(const NonLinearMaltiNDOFHandEyeOptimiser&);

private:
  niftk::NonLinearMaltiNDOFHandEyeCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
