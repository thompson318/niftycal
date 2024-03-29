/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearTsai3ParamOptimiser_h
#define niftkNonLinearTsai3ParamOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearTsai3ParamCostFunction.h"

namespace niftk
{

/**
* \class NonLinearTsai3ParamOptimiser
* \brief Optimises Tz, f and k using niftk::NonLinearTsai3ParamHandCostFunction.
*/
class NonLinearTsai3ParamOptimiser : public itk::Object
{

public:

  typedef  NonLinearTsai3ParamOptimiser Self;
  typedef  itk::Object                  Superclass;
  typedef  itk::SmartPointer<Self>      Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points); // Must be a list of length 1.
  void SetSx(const double& sx);
  void SetK1(const double& k1);
  void SetExtrinsic(const cv::Matx44d* extrinsic);
  void SetIntrinsic(const cv::Mat* const intrinsic);
  double Optimise(double& Tz, double& f, double &k1);

protected:

  NonLinearTsai3ParamOptimiser(); // deliberately protected.
  virtual ~NonLinearTsai3ParamOptimiser(); // deliberately protected.

  NonLinearTsai3ParamOptimiser(const NonLinearTsai3ParamOptimiser&); // deliberately not implemented
  NonLinearTsai3ParamOptimiser& operator=(const NonLinearTsai3ParamOptimiser&); // deliberately not implemented

private:
  niftk::NonLinearTsai3ParamCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
