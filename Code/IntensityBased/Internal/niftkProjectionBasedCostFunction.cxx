/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkProjectionBasedCostFunction.h"
#include <niftkNiftyCalExceptionMacro.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>

namespace niftk
{

//-----------------------------------------------------------------------------
ProjectionBasedCostFunction::ProjectionBasedCostFunction()
{
}


//-----------------------------------------------------------------------------
ProjectionBasedCostFunction::~ProjectionBasedCostFunction()
{
}


//-----------------------------------------------------------------------------
void ProjectionBasedCostFunction::Initialise(const cv::Size2i& windowSize, const std::string& model)
{
  if (windowSize.width <= 0)
  {
    niftkNiftyCalThrow() << "Invalid width.";
  }

  if (windowSize.height <= 0)
  {
    niftkNiftyCalThrow() << "Invalid height.";
  }

  vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
  reader->SetFileName(model.c_str());
  reader->Update();

  if (reader->GetOutput() == nullptr)
  {
    niftkNiftyCalThrow() << "Invalid Poly Data:" << model;
  }
  if (reader->GetOutput()->GetPoints() == nullptr)
  {
    niftkNiftyCalThrow() << "No points found in:" << model;
  }

  vtkPoints* points = reader->GetOutput()->GetPoints();

  double point[3];
  cv::Point3f modelPoint;

  for (int i = 0; i < points->GetNumberOfPoints(); i++)
  {
    points->GetPoint(i, point);

    // We create 4 points per point.
    modelPoint.x = point[0] - 0.1;
    modelPoint.y = point[1] - 0.1;
    modelPoint.z = point[2];
    m_Model.push_back(modelPoint);

    modelPoint.x = point[0] + 0.1;
    modelPoint.y = point[1] - 0.1;
    modelPoint.z = point[2];
    m_Model.push_back(modelPoint);

    modelPoint.x = point[0] + 0.1;
    modelPoint.y = point[1] + 0.1;
    modelPoint.z = point[2];
    m_Model.push_back(modelPoint);

    modelPoint.x = point[0] - 0.1;
    modelPoint.y = point[1] + 0.1;
    modelPoint.z = point[2];
    m_Model.push_back(modelPoint);
  }
}


//-----------------------------------------------------------------------------
unsigned int ProjectionBasedCostFunction::BiLinearInterpolate(const cv::Mat& image, cv::Point2f& pixel) const
{
  unsigned int result = 0;

  int x = static_cast<int>(pixel.x);
  int y = static_cast<int>(pixel.y);

  if (   x >= 0
      && y >= 0
      && x < image.cols - 1
      && y < image.rows - 1
     )
  {
    float dx = pixel.x - x;
    float dy = pixel.y - y;

    float weightTL = (1.0 - dx) * (1.0 - dy);
    float weightTR = (dx)       * (1.0 - dy);
    float weightBL = (1.0 - dx) * (dy);
    float weightBR = (dx)       * (dy);

    float value = static_cast<float>(image.at<unsigned char>(y,   x))   * weightTL
                + static_cast<float>(image.at<unsigned char>(y,   x+1)) * weightTR
                + static_cast<float>(image.at<unsigned char>(y+1, x))   * weightBL
                + static_cast<float>(image.at<unsigned char>(y+1, x+1)) * weightBR;

    result = static_cast<unsigned int>(value);

  }

  return result;
}


//-----------------------------------------------------------------------------
void ProjectionBasedCostFunction::AccumulateSamples(const cv::Mat& greyScaleVideoImageA,
                                                    const cv::Mat& intrinsicsA,
                                                    const cv::Mat& distortionA,
                                                    const cv::Mat& rvecA,
                                                    const cv::Mat& tvecA,
                                                    const cv::Mat& greyScaleVideoImageB,
                                                    const cv::Mat& intrinsicsB,
                                                    const cv::Mat& distortionB,
                                                    const cv::Mat& rvecB,
                                                    const cv::Mat& tvecB,
                                                    unsigned long int& counter,
                                                    cv::Mat& histogramRows,
                                                    cv::Mat& histogramCols,
                                                    cv::Mat& jointHistogram
                                                   ) const
{
  if (greyScaleVideoImageA.size() != greyScaleVideoImageB.size())
  {
    niftkNiftyCalThrow() << "Images have different size.";
  }

  for (int i = 0; i < m_Model.size(); i++)
  {
    std::vector<cv::Point2f> projectedA;
    std::vector<cv::Point2f> projectedB;
    std::vector<cv::Point3f> modelPoint;
    modelPoint.push_back(m_Model[i]);

    // Just doing one at a time for now.
    cv::projectPoints(modelPoint,
                      rvecA,
                      tvecA,
                      intrinsicsA,
                      distortionA,
                      projectedA);

    cv::projectPoints(modelPoint,
                      rvecB,
                      tvecB,
                      intrinsicsB,
                      distortionB,
                      projectedB);

    // Check both points project inside images.
    if (   projectedA[0].x > 0
        && projectedA[0].x < (greyScaleVideoImageA.cols - 1)
        && projectedA[0].y > 0
        && projectedA[0].y < (greyScaleVideoImageA.rows - 1)
        && projectedB[0].x > 0
        && projectedB[0].x < (greyScaleVideoImageB.cols - 1)
        && projectedB[0].y > 0
        && projectedB[0].y < (greyScaleVideoImageB.rows - 1)
       )
    {
      unsigned int a = this->BiLinearInterpolate(greyScaleVideoImageA, projectedA[0]) / 16;
      unsigned int b = this->BiLinearInterpolate(greyScaleVideoImageB, projectedB[0]) / 16;

      jointHistogram.at<double>(a, b) += 1;
      histogramRows.at<double>(a, 0) += 1;
      histogramCols.at<double>(0, b) += 1;
      counter += 1;
    }
  }
}

} // end niftk
