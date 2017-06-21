/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#ifndef niftkCalibratedRenderingPipeline_h
#define niftkCalibratedRenderingPipeline_h

#include "niftkWin32ExportHeader.h"
#include "niftkCalibratedCamera.h"

#include <vtkObject.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkMatrixToLinearTransform.h>
#include <vtkPNGReader.h>
#include <vtkTexture.h>

#include <cv.h>

namespace niftk
{

/**
 * \class CalibratedRenderingPipeline
 * \brief Generates a texture (e.g. calibration image) onto
 * a vtkPolyData model, and renders it with a calibrated camera.
 *
 * Note:
 */
class NIFTYCAL_WINEXPORT CalibratedRenderingPipeline {

public:

  CalibratedRenderingPipeline(
    const cv::Size2i& windowSize,           // e.g. 1920x1080
    const cv::Size2i& calibratedWindowSize, // normally, 1920x1080, but could be scaled such as 1920x540.
    const std::string& modelFileName,
    const std::string& textureFileName,
    const bool& useInterpolation = true
  );

  virtual ~CalibratedRenderingPipeline();

  void ConnectToRenderWindow(vtkRenderWindow* w);

  /**
   * \brief Basically, activated means the actor is connected to renderer,
   * and de-activated means the actor is not connected to renderer.
   */
  void SetActivated(const bool& isActivated);

  /**
  * \brief Updates the whole VTK pipeline.
  */
  virtual void Render();

  /**
  * \brief Calls Render() and writes to specified file.
  * \param fileName file name
  */
  void DumpScreen(const std::string fileName);

  /**
   * \brief Overriden version of above, to copy output direct to OpenCV image.
   *
   * So, if you DumpScreen(filename), or DumpScreen(image) and then cv::imwrite image
   * the result must be identical. i.e. We copy the output into OpenCV buffer, and then
   * if you save the image, its equivalent to saving via VTK.
   */
  void CopyScreen(cv::Mat& image);

  /**
   * \brief Sets background colour, values should be zero to 255.
   */
  void SetBackgroundColour(const unsigned char& red,
                           const unsigned char& green,
                           const unsigned char& blue
                           );
  /**
  * \brief Sets the intrinsic parameters on the camera.
  *
  * This is the 3x3 matrix that you would get from OpenCV calibration.
  */
  void SetIntrinsics(const cv::Mat& intrinsics);

  /**
  * \brief Multiplies model vtkPolyData points by modelToWorld to give a position
  * in world coordinates, having the effect of moving the model relative to the world coordinates.
  *
  * \param modelToWorld rigid body transform
  */
  void SetModelToWorldMatrix(const cv::Matx44d& modelToWorld);

  /**
  * \brief Used to set the camera position in world coordinates.
  * \param cameraToWorld rigid body transform
  *
  * This is the camera extrinsic matrix you get from OpenCV calibration.
  * From a pure calibration perspective, the chessboard, or similar model IS
  * the world coordinate system.
  */
  void SetWorldToCameraMatrix(const cv::Matx44d& worldToCamera);

  /**
   * \brief Defaults to Identity.
   * \param leftToRight rigid body transform
   */
  void SetLeftToRightMatrix(const cv::Matx44d& leftToRight);

private:

  CalibratedRenderingPipeline(const CalibratedRenderingPipeline&);  // Purposefully not implemented.
  void operator=(const CalibratedRenderingPipeline&);  // Purposefully not implemented.

  void UpdateCamera();
  void OpenCVToVTK(const cv::Matx44d& openCV, vtkMatrix4x4& vtk);

  cv::Size2i                                   m_WindowSize;
  cv::Size2i                                   m_CalibratedWindowSize;
  cv::Vec2d                                    m_AspectRatio;

  cv::Mat                                      m_IntrinsicMatrix;
  cv::Matx44d                                  m_LeftToRightMatrix;
  cv::Matx44d                                  m_WorldToCameraMatrix;
  cv::Matx44d                                  m_CameraMatrix;

  vtkSmartPointer<CalibratedCamera>            m_Camera;

  vtkSmartPointer<vtkMatrix4x4>                m_ModelToWorldMatrix;
  vtkSmartPointer<vtkMatrixToLinearTransform>  m_ModelToWorldTransform;

  vtkSmartPointer<vtkPNGReader>                m_TextureReader;
  vtkSmartPointer<vtkTexture>                  m_Texture;

  vtkSmartPointer<vtkPolyDataReader>           m_ModelReader;
  vtkSmartPointer<vtkTransformPolyDataFilter>  m_ModelTransformFilter;
  vtkSmartPointer<vtkPolyDataMapper>           m_ModelMapper;
  vtkSmartPointer<vtkActor>                    m_ModelActor;

  vtkSmartPointer<vtkRenderer>                 m_Renderer;
  vtkRenderWindow*                             m_RenderWindow;

  cv::Mat                                      m_RGBImage;
  cv::Mat                                      m_BGRImage;
};

} // end namepace

#endif
