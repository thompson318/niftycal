/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearNDOFHandEyeOptimiser_h
#define niftkNonLinearNDOFHandEyeOptimiser_h

#include <niftkWin32ExportHeader.h>
#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearNDOFHandEyeCostFunction.h"

namespace niftk
{

/**
* \class NonLinearNDOFHandEyeOptimiser
* \brief Optimises camera extrinsic, hand-eye and model-to-world,
* as an extension of <a href="http://dx.doi.org/10.1002/rcs.1478">Malti 2013</a>.
*/
class NIFTYCAL_WINEXPORT NonLinearNDOFHandEyeOptimiser : public itk::Object
{

public:

  typedef  NonLinearNDOFHandEyeOptimiser Self;
  typedef  itk::Object                        Superclass;
  typedef  itk::SmartPointer<Self>            Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points);
  void SetHandMatrices(const std::list<cv::Matx44d>* const matrices);
  void SetIntrinsic(const cv::Mat* const intrinsic);
  void SetDistortion(const cv::Mat* const distortion);
  double Optimise(cv::Matx44d& modelToWorld,
                  cv::Matx44d& handEye
                 );

protected:

  NonLinearNDOFHandEyeOptimiser();
  virtual ~NonLinearNDOFHandEyeOptimiser();

  NonLinearNDOFHandEyeOptimiser(const NonLinearNDOFHandEyeOptimiser&);
  NonLinearNDOFHandEyeOptimiser& operator=(const NonLinearNDOFHandEyeOptimiser&);

private:
  niftk::NonLinearNDOFHandEyeCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
