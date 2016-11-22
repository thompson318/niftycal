/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai8ParamOptimiser_h
#define niftkNonLinearTsai8ParamOptimiser_h

#include <niftkWin32ExportHeader.h>
#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearTsai8ParamCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai8ParamOptimiser
* \brief Optimises Rx, Ry, Rz, Tx, Ty, Tz, f and k using niftk::NonLinearTsai8ParamHandCostFunction.
* \see Tsai 1987 paper "A Versatile Camera Calibration Techniaue for
* High-Accuracy 3D Machine Vision Metrology Using Off-the-shelf TV Cameras and Lenses",
* IEEE JOURNAL OF ROBOTICS AND AUTOMATION, VOL. RA-3, NO. 4, AUGUST 1987
*/
class NIFTYCAL_WINEXPORT NonLinearTsai8ParamOptimiser : public itk::Object
{

public:

  typedef  NonLinearTsai8ParamOptimiser Self;
  typedef  itk::Object                  Superclass;
  typedef  itk::SmartPointer<Self>      Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points); // Must be a list of length 1.
  void SetIntrinsic(const cv::Mat* const intrinsic);

  double Optimise(double& Rx, double& Ry, double& Rz,
                  double& Tx, double& Ty, double& Tz,
                  double& f, double& k1);

protected:

  NonLinearTsai8ParamOptimiser(); // deliberately protected.
  virtual ~NonLinearTsai8ParamOptimiser(); // deliberately protected.

  NonLinearTsai8ParamOptimiser(const NonLinearTsai8ParamOptimiser&); // deliberately not implemented
  NonLinearTsai8ParamOptimiser& operator=(const NonLinearTsai8ParamOptimiser&); // deliberately not implemented

private:
  niftk::NonLinearTsai8ParamCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
