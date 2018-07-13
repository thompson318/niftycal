/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearMonoCameraCalibration3DOptimiser_h
#define niftkNonLinearMonoCameraCalibration3DOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearMonoCameraCalibration3DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearMonoCameraCalibration3DOptimiser
* \brief Optimises camera intrinsic and extrinsic parameters using 3D reconstruction error.
*
* \see niftk::NonLinearMonoCameraCalibration3DCostFunction
*/
class NonLinearMonoCameraCalibration3DOptimiser : public itk::Object
{

public:

  typedef  NonLinearMonoCameraCalibration3DOptimiser Self;
  typedef  itk::Object                               Superclass;
  typedef  itk::SmartPointer<Self>                   Pointer;
  itkNewMacro(Self);

  void SetModel(const Model3D* const model);
  void SetPoints(const std::list<PointSet>* const points);
  void SetIntrinsic(const cv::Mat& intrinsic);
  void SetDistortion(const cv::Mat& distortion);

  double Optimise(std::vector<cv::Mat>& rvecs,
                  std::vector<cv::Mat>& tvecs
                 );

protected:

  NonLinearMonoCameraCalibration3DOptimiser();
  virtual ~NonLinearMonoCameraCalibration3DOptimiser();

  NonLinearMonoCameraCalibration3DOptimiser(const NonLinearMonoCameraCalibration3DOptimiser&); // Purposefully not implemented.
  NonLinearMonoCameraCalibration3DOptimiser& operator=(const NonLinearMonoCameraCalibration3DOptimiser&); // Purposefully not implemented.

private:
  niftk::NonLinearMonoCameraCalibration3DCostFunction::Pointer m_CostFunction;
};

} // end namespace

#endif
