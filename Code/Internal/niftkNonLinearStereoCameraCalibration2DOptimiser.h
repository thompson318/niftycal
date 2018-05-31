/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearStereoCameraCalibration2DOptimiser_h
#define niftkNonLinearStereoCameraCalibration2DOptimiser_h

#include <itkObject.h>
#include <itkObjectFactory.h>
#include "niftkNonLinearStereoCameraCalibration2DCostFunction.h"

namespace niftk
{

/**
* \class NonLinearStereoCameraCalibration2DOptimiser
* \brief Optimises camera stereo intrinsic and extrinsic parameters.
*/
class NonLinearStereoCameraCalibration2DOptimiser : public itk::Object
{

public:

  typedef  NonLinearStereoCameraCalibration2DOptimiser Self;
  typedef  itk::Object                                 Superclass;
  typedef  itk::SmartPointer<Self>                     Pointer;
  itkNewMacro(Self);
  itkTypeMacro(niftk::NonLinearStereoCameraCalibration2DOptimiser, itk::Object);

  void SetOptimiseIntrinsics(const bool& optimise);
  itkGetConstMacro(OptimiseIntrinsics, bool);

  void SetOptimiseExtrinsics(const bool& optimise);
  itkGetConstMacro(OptimiseExtrinsics, bool);

  void SetOptimiseR2L(const bool& optimise);
  itkGetConstMacro(OptimiseR2L, bool);

  itkSetMacro(ForceUnitVectorAxes, bool);
  itkGetConstMacro(ForceUnitVectorAxes, bool);

  void SetOptimise2DOFStereo(const bool& optimise);
  itkGetConstMacro(Optimise2DOFStereo, bool);

  void SetModelAndPoints(const Model3D* const model,
                         const std::list<PointSet>* const leftPoints,
                         const std::list<PointSet>* const rightPoints
                        );

  void SetIntrinsic(const cv::Mat* const intrinsic);
  void SetDistortion(const cv::Mat* const distortion);

  void SetRightIntrinsic(const cv::Mat* const intrinsic);
  void SetRightDistortion(const cv::Mat* const distortion);

  void SetExtrinsics(const std::vector<cv::Mat>& rvecsLeft,
                     const std::vector<cv::Mat>& tvecsLeft);

  /**
  * \brief Optimises just extrinsic parameters, and returns the 2D RMS projection error.
  *
  * In addition, if Optimise2DOFStereo is true (default is false), then
  * we only optimise 2 DOF of the left-to-right transform, namely
  * x and y translation. The specifc axes of translation
  * are deduced from the initial values passed in for leftToRightRotationMatrix
  * and leftToRightTranslationVector.
  *
  * Furthermore, if Force2DOFAxes is true (default is false), then
  * for this 2DOF optimisation, the axes are forced to be the cameras
  * x and y axes, and rotational parameters are reset.
  *
  * Note: You probably need a very good calibration before calling this.
  */
  double Optimise(cv::Mat& leftIntrinsic,
                  cv::Mat& rightIntrinsic,
                  std::vector<cv::Mat>& rvecsLeft,
                  std::vector<cv::Mat>& tvecsLeft,
                  cv::Mat& leftToRightRotationMatrix,
                  cv::Mat& leftToRightTranslationVector
                 );

protected:

  NonLinearStereoCameraCalibration2DOptimiser();
  virtual ~NonLinearStereoCameraCalibration2DOptimiser();

  NonLinearStereoCameraCalibration2DOptimiser(const NonLinearStereoCameraCalibration2DOptimiser&);
  NonLinearStereoCameraCalibration2DOptimiser& operator=(const NonLinearStereoCameraCalibration2DOptimiser&);

private:
  niftk::NonLinearStereoCameraCalibration2DCostFunction::Pointer m_CostFunction;
  bool                                                           m_OptimiseIntrinsics;
  bool                                                           m_OptimiseExtrinsics;
  bool                                                           m_OptimiseR2L;
  bool                                                           m_Optimise2DOFStereo;
  bool                                                           m_ForceUnitVectorAxes;
};

} // end namespace

#endif
