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
* \brief Optimises R1, R2, R3 (Rodrigues), Tx, Ty, Tz, f, k, Cx, Cy and sx using niftk::NonLinearTsai11ParamHandCostFunction.
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
  void SetSx(const double& sx);
  void SetK1(const double& k1);
  void SetExtrinsic(const cv::Matx44d* extrinsic);
  void SetIntrinsic(const cv::Mat* const intrinsic);
  double Optimise(double& Rx, double& Ry, double& Rz,
                  double& Tx, double& Ty, double& Tz,
                  double& f,
                  double& Cx, double& Cy,
                  double& k1, double& sx
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
