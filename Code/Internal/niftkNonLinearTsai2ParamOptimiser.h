/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai2ParamOptimiser_h
#define niftkNonLinearTsai2ParamOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearTsai2ParamCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai2ParamOptimiser
* \brief Optimises Cx and Cy using niftk::NonLinearTsai2ParamHandCostFunction.
*/
class NonLinearTsai2ParamOptimiser : public itk::Object
{

public:

  typedef  NonLinearTsai2ParamOptimiser Self;
  typedef  itk::Object                  Superclass;
  typedef  itk::SmartPointer<Self>      Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points); // Must be a list of length 1.
  void SetSx(const double& sx);
  void SetK1(const double& k1);
  void SetExtrinsic(const cv::Matx44d* extrinsic);
  void SetIntrinsic(const cv::Mat* const intrinsic);
  double Optimise(double& Cx, double& Cy);

protected:

  NonLinearTsai2ParamOptimiser(); // deliberately protected.
  virtual ~NonLinearTsai2ParamOptimiser(); // deliberately protected.

  NonLinearTsai2ParamOptimiser(const NonLinearTsai2ParamOptimiser&); // deliberately not implemented
  NonLinearTsai2ParamOptimiser& operator=(const NonLinearTsai2ParamOptimiser&); // deliberately not implemented

private:
  niftk::NonLinearTsai2ParamCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
