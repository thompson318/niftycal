/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkIOptimiser_h
#define niftkIOptimiser_h

#include "niftkWin32ExportHeader.h"

namespace niftk
{

/**
* \class IOptimiser
* \brief Interface for any optimiser that can do calibration,
* e.g. Levenberg-Marquardt.
*/
class NIFTYCAL_WINEXPORT IOptimiser {

public:

  IOptimiser();
  virtual ~IOptimiser();

};

} // end namespace

#endif
