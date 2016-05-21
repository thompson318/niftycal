/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearHandEyeOptimiser_h
#define niftkNonLinearHandEyeOptimiser_h

#include "niftkWin32ExportHeader.h"
#include <itkObject.h>
#include <itkObjectFactory.h>
#include <niftkNiftyCalTypes.h>
#include <niftkNonLinearHandEyeCostFunction.h>

namespace niftk
{

/**
* \brief Optimises camera intrinsic, hand-eye and model-to-world
* as per Malti 2013 paper.
*
* Does non-linear (Levenberg-Marquart) optimisation as per Malti 2013
* paper <a href="http://dx.doi.org/10.1002/rcs.1478">
* Hand-eye and radial distortion calibration for rigid endoscopes</a>.
*
* \see niftk::NonLinearHandEyeCostFunction
*/
class NIFTYCAL_WINEXPORT NonLinearHandEyeOptimiser : public itk::Object
{

public:

  typedef  NonLinearHandEyeOptimiser Self;
  typedef  itk::Command              Superclass;
  typedef  itk::SmartPointer<Self>   Pointer;
  itkNewMacro(Self);

  void SetModel(Model3D* const model);
  void SetPoints(std::list<PointSet>* const points);
  void SetHandMatrices(std::list<cv::Matx44d>* const matrices);
  double Optimise(cv::Matx44d& modelToWorld,
                  cv::Matx44d& handEye,
                  cv::Mat& intrinsic,
                  cv::Mat& distortion
                 );

protected:

  NonLinearHandEyeOptimiser();
  virtual ~NonLinearHandEyeOptimiser();

  NonLinearHandEyeOptimiser(const NonLinearHandEyeOptimiser&); // Purposefully not implemented.
  NonLinearHandEyeOptimiser& operator=(const NonLinearHandEyeOptimiser&); // Purposefully not implemented.

private:
  niftk::NonLinearHandEyeCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
