/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "catch.hpp"
#include "niftkCatchMain.h"
#include <niftkIOUtilities.h>
#include <niftkPointUtilities.h>
#include <niftkNiftyCalExceptionMacro.h>

TEST_CASE( "Elvis Hand Eye", "[HandEye]" ) {

  if (niftk::argc < 21)
  {
    std::cerr << "Usage: niftkElvisHandEye 3DPoints.txt 2DPoints.txt rx ry rz tx ty tz mat_1.txt mat_2.txt ... mat_n.txt" << std::endl;
    REQUIRE( niftk::argc >= 21);
  }
}
