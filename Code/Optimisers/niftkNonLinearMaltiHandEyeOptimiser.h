/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMaltiHandEyeOptimiser_h
#define niftkNonLinearMaltiHandEyeOptimiser_h

#include <niftkWin32ExportHeader.h>
#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearMaltiHandEyeCostFunction.h"

namespace niftk
{

/**
* \class NonLinearMaltiHandEyeOptimiser
* \brief Optimises camera intrinsic, hand-eye and model-to-world
* as per <a href="http://dx.doi.org/10.1002/rcs.1478">Malti 2013</a>.
*
* \see niftk::NonLinearMaltiHandEyeCostFunction
*/
class NIFTYCAL_WINEXPORT NonLinearMaltiHandEyeOptimiser : public itk::Object
{

public:

  typedef  NonLinearMaltiHandEyeOptimiser Self;
  typedef  itk::Object                    Superclass;
  typedef  itk::SmartPointer<Self>        Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points);
  void SetHandMatrices(const std::list<cv::Matx44d>* const matrices);
  double Optimise(cv::Matx44d& modelToWorld,
                  cv::Matx44d& handEye,
                  cv::Mat& intrinsic,
                  cv::Mat& distortion
                 );

protected:

  NonLinearMaltiHandEyeOptimiser();
  virtual ~NonLinearMaltiHandEyeOptimiser();

  NonLinearMaltiHandEyeOptimiser(const NonLinearMaltiHandEyeOptimiser&); // Purposefully not implemented.
  NonLinearMaltiHandEyeOptimiser& operator=(const NonLinearMaltiHandEyeOptimiser&); // Purposefully not implemented.

private:
  niftk::NonLinearMaltiHandEyeCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
