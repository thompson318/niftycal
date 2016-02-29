/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkIOUtilities.h"
#include "niftkNiftyCalExceptionMacro.h"

namespace niftk {

//-----------------------------------------------------------------------------
void SavePointSet(const PointSet& p, const std::string& fileName)
{

}


//-----------------------------------------------------------------------------
PointSet LoadPointSet(const std::string& fileName)
{

}


//-----------------------------------------------------------------------------
void SaveModel3D(const Model3D& m, const std::string& fileName)
{

}


//-----------------------------------------------------------------------------
Model3D LoadModel3D(const std::string& fileName)
{

}


//-----------------------------------------------------------------------------
void SaveMatrix(const cv::Mat& m, const std::string& fileName)
{

}


//-----------------------------------------------------------------------------
cv::Mat LoadMatrix(const std::string& fileName)
{

}


//-----------------------------------------------------------------------------
void SavePoints(const bool& all,
                const Model3D& m,
                const std::vector<PointSet>& p,
                const std::string& fileName
                )
{

}


//-----------------------------------------------------------------------------
void LoadPoints(const std::string& fileName,
                const Model3D& m,
                const std::vector<PointSet>& p
                )
{

}

} // end namespace
