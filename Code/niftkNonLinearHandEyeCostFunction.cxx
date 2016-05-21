/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "niftkNonLinearHandEyeCostFunction.h"
#include "niftkNiftyCalExceptionMacro.h"
#include "niftkMatrixUtilities.h"
#include "niftkPointUtilities.h"

namespace niftk
{

//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::NonLinearHandEyeCostFunction()
: m_Model(nullptr)
, m_Points(nullptr)
, m_HandMatrices(nullptr)
, m_NumberOfParameters(0)
, m_NumberOfValues(0)
{
}


//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::~NonLinearHandEyeCostFunction()
{
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::SetModel(Model3D* const model)
{
  if (model == nullptr)
  {
    niftkNiftyCalThrow() << "Model is NULL.";
  }

  m_Model = model;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::SetPoints(std::list<PointSet>* const points)
{
  if (points == nullptr)
  {
    niftkNiftyCalThrow() << "Points are NULL.";
  }


  unsigned int num = 0;
  std::list<PointSet>::const_iterator iter;
  for (iter = points->begin();
       iter != points->end();
       ++iter
       )
  {
    num += (*iter).size();
  }

  m_NumberOfValues = num * 2; // For each point, we have deltaX and deltaY.
  m_Points = points;
  this->Modified();
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::SetHandMatrices(std::list<cv::Matx44d>* const matrices)
{
  if (matrices == nullptr)
  {
    niftkNiftyCalThrow() << "Hand matrices are NULL.";
  }

  m_HandMatrices = matrices;
  this->Modified();
}


//-----------------------------------------------------------------------------
unsigned int NonLinearHandEyeCostFunction::GetNumberOfValues(void) const
{
  return m_NumberOfValues;
}


//-----------------------------------------------------------------------------
unsigned int NonLinearHandEyeCostFunction::GetNumberOfParameters() const
{
  return m_NumberOfParameters;
}


//-----------------------------------------------------------------------------
void NonLinearHandEyeCostFunction::GetDerivative(const ParametersType& parameters, DerivativeType& derivative ) const
{
  niftkNiftyCalThrow() << "Not implemented yet, use vnl derivative.";
}


//-----------------------------------------------------------------------------
double NonLinearHandEyeCostFunction::GetRMS(const MeasureType& values) const
{
  double rms = 0;
  if (values.GetSize() == 0)
  {
    return rms;
  }

  for (unsigned int i = 0; i < values.GetSize(); i++)
  {
    rms += (values[i] * values[i]);
  }
  rms /= static_cast<double>(values.GetSize());
  return sqrt(rms);
}


//-----------------------------------------------------------------------------
NonLinearHandEyeCostFunction::MeasureType
NonLinearHandEyeCostFunction::GetValue(const ParametersType& parameters ) const
{
  if (m_Model == nullptr)
  {
    niftkNiftyCalThrow() << "Model is null.";
  }
  if (m_Points == nullptr)
  {
    niftkNiftyCalThrow() << "Extracted points are null.";
  }
  if (m_HandMatrices == nullptr)
  {
    niftkNiftyCalThrow() << "Hand matrices are null.";
  }
  if (m_Points->empty())
  {
    niftkNiftyCalThrow() << "No extracted points.";
  }
  if (m_HandMatrices->empty())
  {
    niftkNiftyCalThrow() << "No tracking matrices.";
  }
  if (m_Points->size() != m_HandMatrices->size())
  {
    niftkNiftyCalThrow() << "Different number of point sets and hand matrices.";
  }

  MeasureType result;
  result.SetSize(this->GetNumberOfValues());

  cv::Mat handEyeRotationVector = cvCreateMat(1, 3, CV_64FC1);
  handEyeRotationVector.at<double>(0, 0) = parameters[0];
  handEyeRotationVector.at<double>(0, 1) = parameters[1];
  handEyeRotationVector.at<double>(0, 2) = parameters[2];

  cv::Mat handEyeTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  handEyeTranslationVector.at<double>(0, 0) = parameters[3];
  handEyeTranslationVector.at<double>(0, 1) = parameters[4];
  handEyeTranslationVector.at<double>(0, 2) = parameters[5];

  cv::Mat modelToWorldRotationVector = cvCreateMat(1, 3, CV_64FC1);
  modelToWorldRotationVector.at<double>(0, 0) = parameters[6];
  modelToWorldRotationVector.at<double>(0, 1) = parameters[7];
  modelToWorldRotationVector.at<double>(0, 2) = parameters[8];

  cv::Mat modelToWorldTranslationVector = cvCreateMat(1, 3, CV_64FC1);
  modelToWorldTranslationVector.at<double>(0, 0) = parameters[9];
  modelToWorldTranslationVector.at<double>(0, 1) = parameters[10];
  modelToWorldTranslationVector.at<double>(0, 2) = parameters[11];

  cv::Mat intrinsic = cvCreateMat(3, 3, CV_64FC1);
  intrinsic.at<double>(0, 0) = parameters[12];
  intrinsic.at<double>(1, 1) = parameters[13];
  intrinsic.at<double>(0, 2) = parameters[14];
  intrinsic.at<double>(1, 2) = parameters[15];

  cv::Mat distortion = cvCreateMat(1, parameters.GetSize() - 16, CV_64FC1);
  for (int i = 0; i < distortion.cols; i++)
  {
    distortion.at<double>(0, i) = parameters[16+i];
  }

  cv::Mat extrinsicRotationVector = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat extrinsicTranslationVector = cvCreateMat(1, 3, CV_64FC1);

  cv::Matx44d modelToWorld = niftk::RodriguesToMatrix(modelToWorldRotationVector, modelToWorldTranslationVector);
  cv::Matx44d handEye = niftk::RodriguesToMatrix(handEyeRotationVector, handEyeTranslationVector);

  NiftyCalIdType id;
  cv::Point3d    modelPoint;
  cv::Point3f    m;
  cv::Point2f    p;
  std::vector<cv::Point3f> model;
  std::vector<cv::Point2f> observed;
  std::vector<cv::Point2f> projected;

  std::list<PointSet>::const_iterator viewIter;
  niftk::PointSet::const_iterator pointIter;
  std::list<cv::Matx44d>::const_iterator matrixIter;

  unsigned int totalPointCounter = 0;

  // Iterating over each image.
  for (viewIter = m_Points->begin(),
       matrixIter = m_HandMatrices->begin();
       viewIter != m_Points->end()
       && matrixIter != m_HandMatrices->end();
       ++viewIter,
       ++matrixIter
       )
  {
    cv::Matx44d handToWorld = (*matrixIter);
    cv::Matx44d worldToHand = handToWorld.inv();
    cv::Matx44d cameraMatrix = handEye * worldToHand * modelToWorld;
    niftk::MatrixToRodrigues(cameraMatrix, extrinsicRotationVector, extrinsicTranslationVector);

    model.resize((*viewIter).size());
    projected.resize((*viewIter).size());
    observed.resize((*viewIter).size());
    unsigned int pointPerViewCounter = 0;

    // Iterating over each point in the current image.
    for (pointIter = (*viewIter).begin();
         pointIter != (*viewIter).end();
         ++pointIter
         )
    {
      id = (*pointIter).first;
      modelPoint = (*m_Model)[id].point;
      m.x = modelPoint.x;
      m.y = modelPoint.y;
      m.z = modelPoint.z;
      model[pointPerViewCounter] = m;

      p.x = (*pointIter).second.point.x;
      p.y = (*pointIter).second.point.y;
      observed[pointPerViewCounter] = p;

      pointPerViewCounter++;
    }

    // Project all points for that image.
    cv::projectPoints(model, extrinsicRotationVector, extrinsicTranslationVector, intrinsic, distortion, projected);

    // Now measure diff.
    for (unsigned int i = 0; i < observed.size(); i++)
    {
      result[totalPointCounter++] = (observed[i].x - projected[i].x);
      result[totalPointCounter++] = (observed[i].y - projected[i].y);
    }
  }

  return result;
}


} // end namespace
