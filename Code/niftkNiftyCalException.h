/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkNiftyCalException_h
#define niftkNiftyCalException_h

#include "niftkWin32ExportHeader.h"
#include <stdexcept>
#include <ostream>
#include <sstream>

namespace niftk {

/**
* \brief Very basic, base exception class.
*/
class NIFTYCAL_WINEXPORT NiftyCalException : public std::exception
{
public:

  NiftyCalException(const std::string& fileName, int lineNumber);
  virtual ~NiftyCalException() throw( );

  std::string GetFileName() const;
  int GetLineNumber() const;

  std::string GetDescription() const;
  void SetDescription(const std::string& desc);
  virtual const char* what() const throw();

  NiftyCalException& operator<<(std::ostream& (*func)(std::ostream&))
  {
    std::ostringstream ss;
    ss << this->GetDescription() << func;
    this->SetDescription(ss.str());
    return *this;
  }

  template <class T> inline NiftyCalException& operator<<(T& data)
  {
    std::ostringstream ss;
    ss << this->GetDescription() << data;
    this->SetDescription(ss.str());
    return *this;
  }

   template <class T> inline NiftyCalException& operator<<(const T& data)
   {
     std::ostringstream ss;
     ss << this->GetDescription() << data;
     this->SetDescription(ss.str());
     return *this;
   }

private:
  std::string m_Description;
  std::string m_FileName;
  int         m_LineNumber;
};

} // namespace niftk

#endif
