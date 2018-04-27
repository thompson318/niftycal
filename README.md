NiftyCal
---------

NiftyCal is an open source software library to perform video camera calibration.

In short, this software uses OpenCV for [Zhang's method][Zhang2000], and additionally provides implementations
of [Tsai's intrinsic method][Tsai87], [Tsai's hand-eye method][Tsai89], [Datta's iterative method][Datta],
[Shahidi's hand-eye method][Shahidi] and a non-linear method similar to [Malti's hand-eye method][Malti].
A full feature list is available once compiled, by running doxygen.

NiftyCal was developed by the [Centre for Medical Image Computing][cmic] at [University College London (UCL)][ucl].

Useful Links
------------------

 - [Mailing list][mailinglist].
 - NiftyCal is part of [NifTK][niftk].
 - [NiftyCal home page][NiftyCalHome].

License
-----------

Copyright (c) 2016, [University College London][ucl].

BSD License, see LICENSE.txt.

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
[citation]: https://doi.org/10.1117/12.2253717
[niftk]: http://www.niftk.org
[NiftyCalHome]: https://cmiclab.cs.ucl.ac.uk/CMIC/NiftyCal
[April]: http://github.com/NifTK/apriltags
[Datta]: http://dx.doi.org/10.1109/ICCVW.2009.5457474
[Shahidi]: http://dx.doi.org/10.1109/TMI.2002.806597
[Tsai89]: http://dx.doi.org/10.1109/70.34770
[Malti]: http://dx.doi.org/10.1002/rcs.1478
[Tsai87]: http://ieeexplore.ieee.org/document/1087109
[Zhang2000]: http://dx.doi.org/10.1109/34.888718
