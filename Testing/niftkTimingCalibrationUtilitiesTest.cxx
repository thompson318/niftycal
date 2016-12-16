/*=============================================================================

  NiftyCal: A software package for camera calibration.

  Copyright (c) University College London (UCL). All rights reserved.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.

  See LICENSE.txt in the top level directory for details.

=============================================================================*/

#include "catch.hpp"
#include "niftkCatchMain.h"
#include <niftkNiftyCalTypes.h>
#include <niftkTimingCalibration.h>
#include <niftkIOUtilities.h>

TEST_CASE( "ProjectTo1DAndNormalise", "[timing]" ) {

  niftk::TimingSample3D t;

  niftk::TimeSamples3D s;
  t.time = 1;
  t.sample.x = 1;
  t.sample.y = 1;
  t.sample.z = 1;
  s.push_back(t);
  t.time = 2;
  t.sample.x = 2;
  t.sample.y = 2;
  t.sample.z = 2;
  s.push_back(t);
  t.time = 3;
  t.sample.x = 3;
  t.sample.y = 3;
  t.sample.z = 3;
  s.push_back(t);
  t.time = 4;
  t.sample.x = 4;
  t.sample.y = 4;
  t.sample.z = 4;
  s.push_back(t);
  t.time = 5;
  t.sample.x = 5;
  t.sample.y = 5;
  t.sample.z = 5;
  s.push_back(t);

  niftk::TimeSamples1D s1D = niftk::ProjectTo1DAndNormalise(s);

  niftk::TimeSamples1D::const_iterator iter;
  for (iter = s1D.begin(); iter != s1D.end(); ++iter)
  {
    REQUIRE((*iter).sample >= 0);
    REQUIRE((*iter).sample <= 1);
    if ((*iter).time == 3)
    {
      REQUIRE((*iter).sample == 0.5);
    }
  }
}


TEST_CASE( "ResampleTimeStampsToMilliseconds", "[timing]" ) {

  niftk::TimingSample1D t;
  niftk::TimeSamples1D ts;

  t.time = 1403087518406060800;
  t.sample = 0;
  ts.push_back(t);

  t.time = 1403087518408060800;
  t.sample = 1;
  ts.push_back(t);

  niftk::TimeMappedSamples1D tm = niftk::ResampleTimeStampsToMilliseconds(ts);

  niftk::TimeMappedSamples1D::const_iterator iter = tm.begin();

  REQUIRE(tm.size() == 3);
  REQUIRE((*iter).first == 1403087518406000000);
  REQUIRE((*iter).second == 0);
  ++iter;
  REQUIRE((*iter).first == 1403087518407000000);
  REQUIRE((*iter).second == 0.5);
  ++iter;
  REQUIRE((*iter).first == 1403087518408000000);
  REQUIRE((*iter).second == 1);
}


TEST_CASE( "ComputeNCC", "[timing]" ) {

  niftk::TimeMappedSamples1D f;
  f.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518406000000, 1));
  f.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518407000000, 2));
  f.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518408000000, 3));
  niftk::TimeMappedSamples1D m;
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518406000000, 1));
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518407000000, 2));
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518408000000, 3));
  REQUIRE(niftk::ComputeNCC(f, m, 0) == 1);

  m.clear();
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518406000000, -1));
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518407000000, -2));
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518408000000, -3));
  REQUIRE(niftk::ComputeNCC(f, m, 0) == -1);

  m.clear();
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518406000000, 2));
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518407000000, 3));
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518408000000, 4));
  REQUIRE(niftk::ComputeNCC(f, m, 0) == 1);

  m.clear();
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518406000000, 2));
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518407000000, 3));
  m.insert(std::pair<niftk::NiftyCalTimeType, double>(1403087518408000000, 3.5));
  REQUIRE(niftk::ComputeNCC(f, m, 0) != 1);

}
