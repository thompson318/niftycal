/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNonLinearHandEyeOptimizer_h
#define niftkNonLinearHandEyeOptimizer_h

#include "niftkWin32ExportHeader.h"
#include <itkObject.h>
#include <itkObjectFactory.h>

namespace niftk
{

/**
* \brief Optimises camera intrinsic, hand-eye, model-to-world
* as per Malti 2013 paper.
*
* Does non-linear (Levenberg-Marquart) optimisation as per Malti 2013
* paper <a href="http://dx.doi.org/10.1002/rcs.1478">
* Hand–eye and radial distortion calibration for rigid endoscopes</a>.
*
* \see niftk::NonLinearHandEyeOptimizer
*/
class NIFTYCAL_WINEXPORT NonLinearHandEyeOptimizer : public itk::Object
{

public:

  typedef  NonLinearHandEyeOptimizer Self;
  typedef  itk::Command              Superclass;
  typedef  itk::SmartPointer<Self>   Pointer;
  itkNewMacro(Self);

protected:

  NonLinearHandEyeOptimizer();
  virtual ~NonLinearHandEyeOptimizer();

  NonLinearHandEyeOptimizer(const NonLinearHandEyeOptimizer&); // Purposefully not implemented.
  NonLinearHandEyeOptimizer& operator=(const NonLinearHandEyeOptimizer&); // Purposefully not implemented.

private:

};

} // end namespace

#endif

