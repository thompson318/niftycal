/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai10ParamOptimiser_h
#define niftkNonLinearTsai10ParamOptimiser_h

#include <niftkWin32ExportHeader.h>
#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearTsai10ParamCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai10ParamOptimiser
* \brief Optimises Rx, Ry, Rz, Tx, Ty, Tz, f, k, Cx and Cy using niftk::NonLinearTsai10ParamHandCostFunction.
* \see Tsai 1987 paper "A Versatile Camera Calibration Techniaue for
* High-Accuracy 3D Machine Vision Metrology Using Off-the-shelf TV Cameras and Lenses",
* IEEE JOURNAL OF ROBOTICS AND AUTOMATION, VOL. RA-3, NO. 4, AUGUST 1987
*/
class NIFTYCAL_WINEXPORT NonLinearTsai10ParamOptimiser : public itk::Object
{

public:

  typedef  NonLinearTsai10ParamOptimiser Self;
  typedef  itk::Object                   Superclass;
  typedef  itk::SmartPointer<Self>       Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points); // Must be a list of length 1.

  double Optimise(double& Rx, double& Ry, double& Rz,
                  double& Tx, double& Ty, double& Tz,
                  double& f,
                  double& Cx, double& Cy,
                  double& k1
                  );

protected:

  NonLinearTsai10ParamOptimiser(); // deliberately protected.
  virtual ~NonLinearTsai10ParamOptimiser(); // deliberately protected.

  NonLinearTsai10ParamOptimiser(const NonLinearTsai10ParamOptimiser&); // deliberately not implemented
  NonLinearTsai10ParamOptimiser& operator=(const NonLinearTsai10ParamOptimiser&); // deliberately not implemented

private:
  niftk::NonLinearTsai10ParamCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
