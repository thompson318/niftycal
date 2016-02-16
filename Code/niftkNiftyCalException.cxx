#/*============================================================================
#
#  NiftyCal: A software package for camera calibration.
#
#  Copyright (c) University College London (UCL). All rights reserved.
#
#  This software is distributed WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
#  See LICENSE.txt in the top level directory for details.
#
#============================================================================*/

#include "niftkNiftyCalException.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NiftyCalException::NiftyCalException(const std::string& fileName,
                                     int lineNumber)
: std::exception()
, m_Description("")
, m_FileName(fileName)
, m_LineNumber(lineNumber)
{
}


//-----------------------------------------------------------------------------
NiftyCalException::~NiftyCalException() throw( )
{

}


//-----------------------------------------------------------------------------
std::string NiftyCalException::GetFileName() const
{
  return m_FileName;
}


//-----------------------------------------------------------------------------
int NiftyCalException::GetLineNumber() const
{
  return m_LineNumber;
}


//-----------------------------------------------------------------------------
std::string NiftyCalException::GetDescription() const
{
  return m_Description;
}


//-----------------------------------------------------------------------------
void NiftyCalException::SetDescription(const std::string& desc)
{
  m_Description = desc;
}


//-----------------------------------------------------------------------------
const char* NiftyCalException::what() const throw()
{
  return m_Description.c_str();
}

} // end namespace
