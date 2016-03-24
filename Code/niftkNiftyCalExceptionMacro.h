/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNiftyCalExceptionMacro_h
#define niftkNiftyCalExceptionMacro_h

#include "niftkNiftyCalException.h"

#define niftkNiftyCalThrow() throw niftk::NiftyCalException(__FILE__,__LINE__)

#endif
