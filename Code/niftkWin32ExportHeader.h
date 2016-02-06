/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkWin32ExportHeader_h
#define niftkWin32ExportHeader_h

#if (defined(_WIN32) || defined(WIN32)) && !defined(NIFTYCAL_STATIC)
  #ifdef NIFTYCAL_WINDOWS_EXPORT
    #define NIFTYCAL_WINEXPORT __declspec(dllexport)
  #else
    #define NIFTYCAL_WINEXPORT __declspec(dllimport)
  #endif  /* NIFTYCAL_WINEXPORT */
#else
/* linux/mac needs nothing */
  #define NIFTYCAL_WINEXPORT
#endif

#endif
