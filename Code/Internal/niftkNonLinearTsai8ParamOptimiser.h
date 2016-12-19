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

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearTsai8ParamCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai8ParamOptimiser
* \brief Optimises R1, R2, R3 (Rodrigues), Tx, Ty, Tz, f and k using niftk::NonLinearTsai8ParamHandCostFunction.
*/
class NonLinearTsai8ParamOptimiser : public itk::Object
{

public:

  typedef  NonLinearTsai8ParamOptimiser Self;
  typedef  itk::Object                  Superclass;
  typedef  itk::SmartPointer<Self>      Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points); // Must be a list of length 1.
  void SetSx(const double& sx);
  void SetK1(const double& k1);
  void SetExtrinsic(const cv::Matx44d* extrinsic);
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
