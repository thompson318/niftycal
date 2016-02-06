/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkOpenCVChessboardPointDetector.h"

namespace niftk
{

//-----------------------------------------------------------------------------
OpenCVChessboardPointDetector::OpenCVChessboardPointDetector(cv::Size2i numberOfCorners)
: m_NumbeOfCorners(numberOfCorners)
{
}


//-----------------------------------------------------------------------------
OpenCVChessboardPointDetector::~OpenCVChessboardPointDetector()
{

}


//-----------------------------------------------------------------------------
std::vector< Point2D > OpenCVChessboardPointDetector::GetPoints()
{
  std::vector< Point2D > result;
  return result;
}

} // end namespace
