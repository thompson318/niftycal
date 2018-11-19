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
#include "niftkMatrixUtilities.h"

#include <iostream>
#include <fstream>
#include <sstream>

namespace niftk {

//-----------------------------------------------------------------------------
void SavePointSet(const PointSet& p, const std::string& fileName)
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ofstream ofs;
  ofs.open (fileName, std::ofstream::out);
  if (!ofs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for writing.";
  }

  PointSet::const_iterator iter;
  for (iter = p.begin(); iter != p.end(); ++iter)
  {
    ofs << (*iter).first << " " << (*iter).second.point.x << " " << (*iter).second.point.y << std::endl;
  }

  ofs.close();
}


//-----------------------------------------------------------------------------
PointSet LoadPointSet(const std::string& fileName)
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ifstream ifs;
  ifs.open (fileName, std::ofstream::in);
  if (!ifs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for reading.";
  }

  niftk::PointSet result;

  std::string line;
  while (std::getline(ifs, line))
  {
    std::stringstream ss;
    ss << line;

    Point2D tmp;
    ss >> tmp.id;
    ss >> tmp.point.x;
    ss >> tmp.point.y;
    if (!ifs.bad() && !ifs.fail())
    {
      result.insert(IdPoint2D(tmp.id, tmp));
    }
  }

  if (ifs.bad())
  {
    niftkNiftyCalThrow() << "Failed to read file:" << fileName << std::endl;
  }

  ifs.close();

  return result;
}


//-----------------------------------------------------------------------------
void SaveModel3D(const Model3D& m, const std::string& fileName)
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ofstream ofs;
  ofs.open (fileName, std::ofstream::out);
  if (!ofs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for writing.";
  }

  Model3D::const_iterator iter;
  for (iter = m.begin(); iter != m.end(); ++iter)
  {
    ofs << (*iter).first << " "
        << (*iter).second.point.x << " "
        << (*iter).second.point.y << " "
        << (*iter).second.point.z << std::endl;
  }

  ofs.close();
}


//-----------------------------------------------------------------------------
Model3D LoadModel3D(const std::string& fileName)
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ifstream ifs;
  ifs.open (fileName, std::ofstream::in);
  if (!ifs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for reading.";
  }

  Model3D result;

  std::string line;
  while (std::getline(ifs, line))
  {
    std::stringstream ss;
    ss << line;

    Point3D tmp;
    ss >> tmp.id;
    ss >> tmp.point.x;
    ss >> tmp.point.y;
    ss >> tmp.point.z;
    if (!ifs.bad() && !ifs.fail())
    {
      result.insert(IdPoint3D(tmp.id, tmp));
    }
  }

  if (ifs.bad())
  {
    niftkNiftyCalThrow() << "Failed to read file:" << fileName << std::endl;
  }

  ifs.close();

  return result;
}


//-----------------------------------------------------------------------------
void SaveMatrix(const cv::Mat& m, const std::string& fileName)
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ofstream ofs;
  ofs.precision(10);
  ofs.width(10);

  ofs.open (fileName, std::ofstream::out);
  if (!ofs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for writing.";
  }

  for (size_t r = 0; r < m.rows; r++)
  {
    for (size_t c = 0; c < m.cols; c++)
    {
      ofs << m.at<double>(r, c) << " ";
    }
    ofs << std::endl;
  }

  ofs.close();
}


//-----------------------------------------------------------------------------
cv::Mat LoadMatrix(const std::string& fileName)
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ifstream ifs;
  ifs.open (fileName, std::ifstream::in);
  if (!ifs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for reading.";
  }

  cv::Mat result;
  std::vector< std::vector<double> > data;

  std::string line;
  while (std::getline(ifs, line))
  {
    line = line.append(" ");
    std::istringstream iss(line);
    std::vector<double> d;
    while(iss.good())
    {
      double a;
      iss >> a;
      if (iss.good())
      {
        d.push_back(a);
      }
    }
    if (d.size() > 0)
    {
      data.push_back(d);
    }
  }
  ifs.close();

  if (data.empty())
  {
    return result;
  }

  unsigned int cols;
  if (data[0].size() == 0)
  {
    niftkNiftyCalThrow() << "First line of matrix had no data.";
  }
  cols = data[0].size();

  result = cv::Mat::zeros(data.size(), cols, CV_64FC1);
  for (int r = 0; r < data.size(); r++)
  {
    if (data[r].size() != cols)
    {
      niftkNiftyCalThrow() << "Row " << r << ", does not contain " << cols << " columns.";
    }
    for (int c = 0; c < data[r].size(); c++)
    {
      result.at<double>(r,c) = data[r][c];
    }
  }

  return result;
}


//-----------------------------------------------------------------------------
void SavePoints(const Model3D& m,
                const std::list<PointSet>& p,
                const std::string& fileName
                )
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ofstream ofs;
  ofs.open (fileName, std::ofstream::out);
  if (!ofs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for writing.";
  }

  size_t counter = 0;
  std::list<PointSet>::const_iterator listIter;
  PointSet::const_iterator pointIter;
  Model3D::const_iterator modelIter;

  for (listIter = p.begin(); listIter != p.end(); ++listIter)
  {
    for (pointIter = (*listIter).begin(); pointIter != (*listIter).end(); ++pointIter)
    {
      NiftyCalIdType id = (*pointIter).first;
      modelIter = m.find(id);

      if (modelIter != m.end())
      {
        ofs << counter << " "
            << id << " "
            << (*modelIter).second.point.x << " "
            << (*modelIter).second.point.y << " "
            << (*modelIter).second.point.z << " "
            << (*pointIter).second.point.x << " "
            << (*pointIter).second.point.y
            << std::endl;
      }
    }
    counter++;
  }

  ofs.close();
}


//-----------------------------------------------------------------------------
void LoadPoints(const std::string& fileName,
                const Model3D& m,
                const std::list<PointSet>& p
                )
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ofstream ifs;
  ifs.open (fileName, std::ofstream::in);
  if (!ifs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for reading.";
  }

  niftkNiftyCalThrow() << "Not implemented yet!";

  ifs.close();
}


//-----------------------------------------------------------------------------
void SaveNifTKIntrinsics(const cv::Mat &intrinsics,
                         const cv::Mat &distortion,
                         const std::string& fileName
                        )
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  if (intrinsics.rows != 3 || intrinsics.cols != 3)
  {
    niftkNiftyCalThrow() << "Invalid intrinsics matrix size.";
  }

  if (distortion.rows != 1)
  {
    niftkNiftyCalThrow() << "Invalid distortion matrix size.";
  }

  std::ofstream ofs;
  ofs.precision(10);
  ofs.width(10);

  ofs.open (fileName, std::ofstream::out);
  if (!ofs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for writing.";
  }

  for (int r = 0; r < intrinsics.rows; r++)
  {
    for (int c = 0; c < intrinsics.cols; c++)
    {
      ofs << intrinsics.at<double>(r, c) << " ";
    }
    ofs << std::endl;
  }
  for (int c = 0; c < distortion.cols; c++)
  {
    ofs << distortion.at<double>(0,c) << " ";
  }
  ofs << std::endl;

  ofs.close();
}


//-----------------------------------------------------------------------------
void SaveNifTKStereoExtrinsics(const cv::Mat& leftToRightRotationMatrix,
                               const cv::Mat& leftToRightTranslationVector,
                               const std::string& fileName
                              )
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  if (leftToRightRotationMatrix.rows != 3 || leftToRightRotationMatrix.cols != 3)
  {
    niftkNiftyCalThrow() << "Invalid rotation matrix size.";
  }

  if (leftToRightTranslationVector.rows != 3 || leftToRightTranslationVector.cols != 1)
  {
    niftkNiftyCalThrow() << "Invalid translation vector size.";
  }

  std::ofstream ofs;
  ofs.precision(10);
  ofs.width(10);

  ofs.open (fileName, std::ofstream::out);
  if (!ofs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for writing.";
  }

  cv::Matx44d mat = niftk::RotationAndTranslationToMatrix(leftToRightRotationMatrix,
                                                          leftToRightTranslationVector);

  // Beware: OpenCV and NiftyCal calculate "left-to-right".
  //         NifTK uses "right-to-left", so here we deliberately invert.
  cv::Matx44d matInv = mat.inv(cv::DECOMP_SVD);

  // And here we deliberately output the inverted matrix.
  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      ofs << matInv(r,c) << " ";
    }
    ofs << std::endl;
  }
  ofs << matInv(0,3) << " " << matInv(1,3) << " " << matInv(2,3) << std::endl;

  ofs.close();
}


//-----------------------------------------------------------------------------
void LoadNifTKStereoExtrinsics(const std::string& fileName,
                               cv::Mat& leftToRightRotationMatrix,
                               cv::Mat& leftToRightTranslationVector
                              )
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  if (leftToRightRotationMatrix.rows != 3 || leftToRightRotationMatrix.cols != 3)
  {
    niftkNiftyCalThrow() << "Invalid rotation matrix size.";
  }

  if (leftToRightTranslationVector.rows != 3 || leftToRightTranslationVector.cols != 1)
  {
    niftkNiftyCalThrow() << "Invalid translation vector size.";
  }

  cv::Mat tmpFromFile = niftk::LoadMatrix(fileName);

  if (tmpFromFile.rows != 4 || tmpFromFile.cols != 3)
  {
    niftkNiftyCalThrow() << "Invalid left to right matrix size.";
  }

  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      leftToRightRotationMatrix.at<double>(r, c) = tmpFromFile.at<double>(r, c);
    }
  }
  for (int r = 0; r < 3; r++)
  {
    leftToRightTranslationVector.at<double>(r, 0) = tmpFromFile.at<double>(3, r);
  }

  // Beware: OpenCV and NiftyCal calculate "left-to-right".
  //         NifTK uses "right-to-left", so here we deliberately invert.

  cv::Matx44d mat = niftk::RotationAndTranslationToMatrix(leftToRightRotationMatrix, leftToRightTranslationVector);
  cv::Matx44d matInv = mat.inv(cv::DECOMP_SVD);

  for (int r = 0; r < 3; r++)
  {
    for (int c = 0; c < 3; c++)
    {
      leftToRightRotationMatrix.at<double>(r, c) = matInv(r, c);
    }
  }
  for (int r = 0; r < 3; r++)
  {
    leftToRightTranslationVector.at<double>(r, 0) = matInv(r, 3);
  }
}


//-----------------------------------------------------------------------------
void SaveRigidParams(const cv::Matx44d& matrix,
                     const std::string& fileName
                    )
{
  cv::Mat rvec = cv::Mat::zeros(1, 3, CV_64FC1);
  cv::Mat tvec = cv::Mat::zeros(1, 3, CV_64FC1);

  niftk::MatrixToRodrigues(matrix, rvec, tvec);
  niftk::SaveRigidParams(rvec, tvec, fileName);
}


//-----------------------------------------------------------------------------
void SaveRigidParams(const cv::Mat& rvec,
                     const cv::Mat& tvec,
                     const std::string& fileName
                    )
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ofstream ofs;
  ofs.precision(10);
  ofs.width(10);

  ofs.open (fileName, std::ofstream::out);
  if (!ofs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for writing.";
  }

  ofs << rvec.at<double>(0, 0) << " "
      << rvec.at<double>(0, 1) << " "
      << rvec.at<double>(0, 2) << " "
      << tvec.at<double>(0, 0) << " "
      << tvec.at<double>(0, 1) << " "
      << tvec.at<double>(0, 2) << std::endl;

  ofs.close();
}


//-----------------------------------------------------------------------------
void Save4x4Matrix(const cv::Mat& leftToRightRotationMatrix,
                   const cv::Mat& leftToRightTranslationVector,
                   const std::string& fileName
                  )
{
  cv::Matx44d mat = niftk::RotationAndTranslationToMatrix(leftToRightRotationMatrix,
                                                          leftToRightTranslationVector);

  niftk::Save4x4Matrix(mat, fileName);
}


//-----------------------------------------------------------------------------
void Save4x4Matrix(const cv::Matx44d& matrix,
                   const std::string& fileName
                  )
{
  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ofstream ofs;
  ofs.precision(10);
  ofs.width(10);

  ofs.open (fileName, std::ofstream::out);
  if (!ofs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for writing.";
  }

  for (int r = 0; r < 4; r++)
  {
    for (int c = 0; c < 4; c++)
    {
      ofs << matrix(r,c) << " ";
    }
    ofs << std::endl;
  }

  ofs.close();
}


//-----------------------------------------------------------------------------
TimeSamples2D LoadTimeSamples2D(const std::string& fileName)
{
  TimeSamples2D result;

  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ifstream ifs;
  ifs.open (fileName, std::ofstream::in);
  if (!ifs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for reading.";
  }

  while (!ifs.eof())
  {
    TimingSample2D tmp;
    ifs >> tmp.time;
    ifs >> tmp.sample.x;
    ifs >> tmp.sample.y;
    if (!ifs.bad() && !ifs.fail())
    {
      result.push_back(tmp);
    }
  }

  ifs.close();

  return result;
}


//-----------------------------------------------------------------------------
TimeSamples3D LoadTimeSamples3D(const std::string& fileName)
{
  TimeSamples3D result;

  if (fileName.size() == 0)
  {
    niftkNiftyCalThrow() << "Empty filename.";
  }

  std::ifstream ifs;
  ifs.open (fileName, std::ofstream::in);
  if (!ifs.is_open())
  {
    niftkNiftyCalThrow() << "Failed to open file:" << fileName << " for reading.";
  }

  while (!ifs.eof())
  {
    TimingSample3D tmp;
    ifs >> tmp.time;
    ifs >> tmp.sample.x;
    ifs >> tmp.sample.y;
    ifs >> tmp.sample.z;
    if (!ifs.bad() && !ifs.fail())
    {
      result.push_back(tmp);
    }
  }

  ifs.close();

  return result;
}

} // end namespace
