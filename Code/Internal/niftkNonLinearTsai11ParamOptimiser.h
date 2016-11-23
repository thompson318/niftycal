/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai11ParamOptimiser_h
#define niftkNonLinearTsai11ParamOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearTsai11ParamCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai11ParamOptimiser
* \brief Optimises Rx, Ry, Rz, Tx, Ty, Tz, f, k, Cx, Cy and sx using niftk::NonLinearTsai11ParamHandCostFunction.
* \see Tsai 1987 paper "A Versatile Camera Calibration Techniaue for
* High-Accuracy 3D Machine Vision Metrology Using Off-the-shelf TV Cameras and Lenses",
* IEEE JOURNAL OF ROBOTICS AND AUTOMATION, VOL. RA-3, NO. 4, AUGUST 1987
*/
class NonLinearTsai11ParamOptimiser : public itk::Object
{

public:

  typedef  NonLinearTsai11ParamOptimiser Self;
  typedef  itk::Object                   Superclass;
  typedef  itk::SmartPointer<Self>       Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points); // Must be a list of length 1.

  double Optimise(double& Rx, double& Ry, double& Rz,
                  double& Tx, double& Ty, double& Tz,
                  double& fx, double& fy,
                  double& Cx, double& Cy,
                  double& k1
                  );

protected:

  NonLinearTsai11ParamOptimiser(); // deliberately protected.
  virtual ~NonLinearTsai11ParamOptimiser(); // deliberately protected.

  NonLinearTsai11ParamOptimiser(const NonLinearTsai11ParamOptimiser&); // deliberately not implemented
  NonLinearTsai11ParamOptimiser& operator=(const NonLinearTsai11ParamOptimiser&); // deliberately not implemented

private:
  niftk::NonLinearTsai11ParamCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
