NiftyCal
---------

NiftyCal is a software library to perform
video camera calibration, largely based on OpenCV.

NiftyCal provides:
 - 2D and 3D structures that stores each point along with its identifier.
 - A simple IPoint2DDetector interface that all point detectors implement.
 - A range of IPoint2DDetector implementations including chessboards, asymmetrical circle grids, AprilTags,
   and template matching of [dots and rings][Dutta].
 - Camera calibration using iterative refinement of control points, as in [Dutta 2009][Dutta].
 - Hand-eye calibration using a direct matrix multiplication method as in [Shahidi 2002][Shahidi].
 - Hand-eye calibration using linear least-squares as in [Tsai 1989][Tsai].
 - Hand-eye calibration using non-linear minimisation as in [Malti 2013][Malti].
 - Performance enhancements using OpenMP.
 - Performance enhancements using CUDA for NVIDIA GPUs (coming soon).

NiftyCal was developed by the [Centre for Medical Image Computing][cmic] at [University College London (UCL)][ucl].

Useful Links
------------------

 - [Mailing list][mailinglist].
 - NiftyCal is part of [NifTK][niftk].
 - [NiftyCal home page][NiftyCalHome].
 - [Dashboard][dashboard].

License
-----------

Copyright (c) 2016, [University College London][ucl].

This project is NOT open-source, and should NOT be distributed
without the prior permission of [Dr. Matt Clarkson][matt].


Supported Platforms
-----------------------------

NiftyCal is a cross-platform C++ library and officially supports:

 - Windows
 - MacOS X
 - Linux


Build Instructions
-----------------------------

For developers:

```
git clone https://cmiclab.cs.ucl.ac.uk/CMIC/NiftyCal.git
mkdir NiftyCal-build
cd NiftyCal-build
cmake -DOpenCV_DIR:PATH=${OpenCV_DIR} -DEigen_DIR:PATH=${Eigen_DIR} -DEigen_INCLUDE_DIR:PATH=${Eigen_INCLUDE_DIR}
```
In addition, AprilTags is optional and can be specified with
```
-DAprilTags_DIRECTORY:PATH=${AprilTags_DIR}
```
and similarly, ITK is optional and can be specified with
```
-DITK_DIR:PATH=${ITK_DIR}
```

where these variable substitutions point to your installed version of these libraries.


Branching Workflow
------------------

 1. Raise issue in Gitlab Issue Tracker.
 2. Create a feature branch called ```<issue-number>-<some-short-description>```
    replacing ```<issue-number>``` with the Gitlab issue number
    and ```<some-short-description>``` with your description of the thing you are implementing.
 3. Code on that branch.
 4. Push to remote when ready.
 5. Create merge request, and assign to Matt Clarkson.
 6. Matt will code review, merge to master and remove the feature branch when it looks ready.

[cmic]: http://cmic.cs.ucl.ac.uk
[ucl]: http://www.ucl.ac.uk
[mailinglist]: https://www.mailinglists.ucl.ac.uk/mailman/listinfo/niftk-users
[dashboard]: http://cdash.cmiclab.cs.ucl.ac.uk/index.php?project=NiftyCal
[citation]: http://link.springer.com/article/10.1007%2Fs11548-014-1124-7
[niftk]: http://www.niftk.org
[NiftyCalHome]: https://cmiclab.cs.ucl.ac.uk/CMIC/NiftyCal
[matt]: http://www.mattclarkson.co.uk
[CMake] : http://www.cmake.org
[April]: http://github.com/NifTK/apriltags
[Dutta]: http://dx.doi.org/10.1109/ICCVW.2009.5457474
[Shahidi]: http://dx.doi.org/10.1109/TMI.2002.806597
[Tsai]: http://dx.doi.org/10.1109/70.34770
[Malti]: http://dx.doi.org/10.1002/rcs.1478
