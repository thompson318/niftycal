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
#include "niftkNiftyCalException.h"
#include "niftkNiftyCalExceptionMacro.h"
#include <iostream>

TEST_CASE( "Extract chessboard points", "[chessboard]" ) {

  int expectedNumberOfArguments =  1;
  if (niftk::argc != expectedNumberOfArguments)
  {
    std::cerr << "Usage: niftkNiftyCalExceptionTest" << std::endl;
    REQUIRE( niftk::argc == expectedNumberOfArguments);
  }

  std::string msg("Help Me!");

  try
  {
    niftkNiftyCalThrow() << msg;
  }
  catch (const niftk::NiftyCalException& e)
  {
    REQUIRE( e.GetDescription() == msg);
    REQUIRE( e.GetLineNumber() == 34);
    REQUIRE( e.GetFileName() == __FILE__);
  }

  try
  {
    niftkNiftyCalThrow() << "Hello " << 1 << " 2 " << 3;
  }
  catch (const niftk::NiftyCalException& e)
  {
    REQUIRE( e.GetDescription() == "Hello 1 2 3");
    REQUIRE( e.GetLineNumber() == 45);
    REQUIRE( e.GetFileName() == __FILE__);
  }

}
