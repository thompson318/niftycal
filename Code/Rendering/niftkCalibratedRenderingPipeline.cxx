/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkCalibratedRenderingPipeline.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>

//-----------------------------------------------------------------------------
CalibratedRenderingPipeline::CalibratedRenderingPipeline(
    const cv::Size2i&  windowSize,           // e.g. 1920x1080
    const cv::Size2i&  calibratedWindowSize, // normally, 1920x1080, but could be scaled such as 1920x540.
    const std::string& modelFileName,
    const std::string& textureFileName
    )
: m_WindowSize(windowSize)
, m_CalibratedWindowSize(calibratedWindowSize)
{
  // Do all validation early, and bail out without doing anything.
  if (windowSize.width <= 0 || windowSize.height <= 0)
  {
    niftkNiftyCalThrow() << "Invalid windowSize:" << windowSize;
  }
  if (calibratedWindowSize.width <= 0 || calibratedWindowSize.height <= 0)
  {
    niftkNiftyCalThrow() << "Invalid windowSize:" << windowSize;
  }
  if (modelFileName.empty())
  {
    niftkNiftyCalThrow() << "Model file name is empty.";
  }
  if (textureFileName.empty())
  {
    niftkNiftyCalThrow() << "Texture file name is empty.";
  }

  m_Camera = vtkSmartPointer<CalibratedCamera>::New();

  m_ModelReader = vtkSmartPointer<vtkPolyDataReader>::New();
  m_ModelReader->SetFileName(modelFileName.c_str());

  m_ModelToWorldMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
  m_ModelToWorldMatrix->Identity();

  m_ModelToWorldTransform = vtkSmartPointer<vtkMatrixToLinearTransform>::New();
  m_ModelToWorldTransform->SetInput(m_ModelToWorldMatrix);

  m_ModelTransformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  m_ModelTransformFilter->SetTransform(m_ModelToWorldTransform);
  m_ModelTransformFilter->SetInputConnection(m_ModelReader->GetOutputPort());

  m_TextureReader = vtkSmartPointer<vtkPNGReader>::New();
  m_TextureReader->SetFileName(textureFileName.c_str());

  m_Texture = vtkSmartPointer<vtkTexture>::New();
  m_Texture->SetInputConnection(m_TextureReader->GetOutputPort());
  m_Texture->InterpolateOff();

  m_ModelMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  m_ModelMapper->SetInputConnection(m_ModelTransformFilter->GetOutputPort());
  m_ModelMapper->ScalarVisibilityOff();

  m_ModelActor = vtkSmartPointer<vtkActor>::New();
  //m_ModelActor->GetProperty()->BackfaceCullingOn();
  m_ModelActor->GetProperty()->SetInterpolationToFlat();
  m_ModelActor->SetTexture(m_Texture);
  m_ModelActor->SetMapper(m_ModelMapper);

  m_Renderer = vtkSmartPointer<vtkRenderer>::New();
  m_Renderer->SetBackground(0, 0, 255);  // RGB
  m_Renderer->AddActor(m_ModelActor);

  m_Renderer->SetActiveCamera(m_Camera);
  m_Renderer->SetLightFollowCamera(true);
  m_Renderer->ResetCamera();
  m_Renderer->InteractiveOff();

  m_IntrinsicMatrix = cv::Mat::eye(3, 3, CV_64FC1);
  m_LeftToRightMatrix = cv::Matx44d::eye();
  m_RightToLeftMatrix = cv::Matx44d::eye();
  m_WorldToCameraMatrix = cv::Matx44d::eye();
  m_CameraMatrix = cv::Matx44d::eye();
}


//-----------------------------------------------------------------------------
CalibratedRenderingPipeline::~CalibratedRenderingPipeline()
{
}


//-----------------------------------------------------------------------------
void CalibratedRenderingPipeline::ConnectToRenderWindow(vtkRenderWindow *w)
{
  m_RenderWindow = w;
  m_RenderWindow->AddRenderer(m_Renderer);
}


//-----------------------------------------------------------------------------
void CalibratedRenderingPipeline::OpenCVToVTK(const cv::Matx44d& openCV, vtkMatrix4x4& vtk)
{
  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 4; c++)
    {
      vtk.SetElement(r, c, openCV(r,c));
    }
  }
}


//-----------------------------------------------------------------------------
void CalibratedRenderingPipeline::Render()
{
  this->UpdateCamera();
  m_RenderWindow->Render();
}


//-----------------------------------------------------------------------------
void CalibratedRenderingPipeline::DumpScreen(const std::string fileName)
{
  this->Render();

  // Keep these local, or else the vtkWindowToImageFilter always appeared to cache its output,
  // regardless of the value of ShouldRerenderOn.  
  vtkSmartPointer<vtkWindowToImageFilter> renderWindowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
  renderWindowToImageFilter->SetInput(m_RenderWindow);
  renderWindowToImageFilter->SetInputBufferTypeToRGB();
  renderWindowToImageFilter->SetMagnification(1);
  renderWindowToImageFilter->ShouldRerenderOn();

  vtkSmartPointer<vtkPNGWriter> renderedImageWriter = vtkSmartPointer<vtkPNGWriter>::New();
  renderedImageWriter->SetInputConnection(renderWindowToImageFilter->GetOutputPort());
  renderedImageWriter->SetFileName(fileName.c_str());
  renderedImageWriter->Write();
}


//-----------------------------------------------------------------------------
void CalibratedRenderingPipeline::SetIntrinsics(const cv::Mat& intrinsics)
{
  m_IntrinsicMatrix = intrinsics;
}


//-----------------------------------------------------------------------------
void CalibratedRenderingPipeline::SetModelToWorldMatrix(const cv::Matx44d& modelToWorld)
{
  this->OpenCVToVTK(modelToWorld, *m_ModelToWorldMatrix);
  m_ModelToWorldMatrix->Modified();
  m_ModelToWorldTransform->Modified();
}


//-----------------------------------------------------------------------------
void CalibratedRenderingPipeline::SetWorldToCameraMatrix(const cv::Matx44d& worldToCamera)
{
  m_WorldToCameraMatrix = worldToCamera;
}


//-----------------------------------------------------------------------------
void CalibratedRenderingPipeline::SetLeftToRightMatrix(const cv::Matx44d& leftToRight)
{
  m_LeftToRightMatrix = leftToRight;
  m_RightToLeftMatrix = leftToRight.inv();
}


//-----------------------------------------------------------------------------
void CalibratedRenderingPipeline::UpdateCamera()
{
  double origin[4]     = {0, 0,    0,    1};
  double focalPoint[4] = {0, 0,    1000, 1};
  double viewUp[4]     = {0, -1000, 0,    1};

  m_AspectRatio[0] = static_cast<double>(m_WindowSize.width)/static_cast<double>(m_CalibratedWindowSize.width);
  m_AspectRatio[1] = static_cast<double>(m_WindowSize.height)/static_cast<double>(m_CalibratedWindowSize.height);

  m_Camera->SetCalibratedImageSize(m_CalibratedWindowSize.width, m_CalibratedWindowSize.height, m_AspectRatio[0]/m_AspectRatio[1]);
  m_Camera->SetIntrinsicParameters(m_IntrinsicMatrix.at<double>(0,0),
                                   m_IntrinsicMatrix.at<double>(1,1),
                                   m_IntrinsicMatrix.at<double>(0,2),
                                   m_IntrinsicMatrix.at<double>(1,2));

  m_Camera->SetUseCalibratedCamera(true);
  m_Camera->SetActualWindowSize(m_WindowSize.width, m_WindowSize.height);

  m_CameraMatrix = (m_WorldToCameraMatrix * m_RightToLeftMatrix).inv();

  vtkSmartPointer<vtkMatrix4x4> tmp = vtkSmartPointer<vtkMatrix4x4>::New();
  this->OpenCVToVTK(m_CameraMatrix, *tmp);

  tmp->MultiplyPoint(origin, origin);
  tmp->MultiplyPoint(focalPoint, focalPoint);
  tmp->MultiplyPoint(viewUp, viewUp);

  viewUp[0] = viewUp[0] - origin[0];
  viewUp[1] = viewUp[1] - origin[1];
  viewUp[2] = viewUp[2] - origin[2];

  m_Camera->SetPosition(origin[0], origin[1], origin[2]);
  m_Camera->SetFocalPoint(focalPoint[0], focalPoint[1], focalPoint[2]);
  m_Camera->SetViewUp(viewUp[0], viewUp[1], viewUp[2]);
  m_Camera->SetClippingRange(2, 5000);
  m_Camera->Modified();
}
