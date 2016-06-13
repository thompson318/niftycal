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

  while (!ifs.eof())
  {
    Point2D tmp;
    ifs >> tmp.id;
    ifs >> tmp.point.x;
    ifs >> tmp.point.y;
    if (!ifs.bad() && !ifs.fail())
    {
      result.insert(IdPoint2D(tmp.id, tmp));
    }
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

  while (ifs.good())
  {
    Point3D tmp;
    ifs >> tmp.id;
    ifs >> tmp.point.x;
    ifs >> tmp.point.y;
    ifs >> tmp.point.z;
    if (ifs.good())
    {
      result.insert(IdPoint3D(tmp.id, tmp));
    }
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

  result = cvCreateMat(data.size(), cols, CV_64FC1);
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
  cv::Matx44d matInv = mat.inv();

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
void SaveRigidParams(const cv::Matx44d& matrix,
                     const std::string& fileName
                    )
{
  cv::Mat rvec = cvCreateMat(1, 3, CV_64FC1);
  cv::Mat tvec = cvCreateMat(1, 3, CV_64FC1);

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

} // end namespace
