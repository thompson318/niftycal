NiftyCal
---------

NiftyCal is an open-source software library to perform
video camera calibration, largely based on OpenCV.

NiftyCal was developed by the [Centre for Medical Image Computing][cmic] at [University College London (UCL)][ucl].

If you use this software, please cite [this paper][citation]. 

Useful Links
------------------

 - [Mailing list][mailinglist].
 - NiftyCal is part of [NifTK][niftk].
 - [NiftyCal home page][NiftyCalHome].
 - [Dashboard][dashboard].
 - [Nightly code documentation][doxygen].

License
-----------

Copyright (c) 2016, [University College London][ucl].

NiftyCal is available as free open-source software under a BSD license.
Other licenses apply for the dependencies.


Supported Platforms
-----------------------------

NiftyCal is a cross-platform C++ library and officially supports:

 - Windows
 - MacOS X
 - Linux


Branching Workflow
------------------

 1. Raise issue in Gitlab Issue Tracker.
 2. Create a feature branch called <issue-number>-some-short-description
    replacing <issue-number> with the Gitlab issue number.
 3. Code on that branch.
 4. Push to remote when ready.
 5. Merge to master when Gitlab-CI is Green.
 6. Remove merged (feature) branch.

An alternative to points 5 and 6 is to assign a merge request, 
and the merger should select to automatically delete the branch
via the web interface.

[cmic]: http://cmic.cs.ucl.ac.uk
[ucl]: http://www.ucl.ac.uk
[mailinglist]: https://www.mailinglists.ucl.ac.uk/mailman/listinfo/niftk-users
[dashboard]: http://cdash.cmiclab.cs.ucl.ac.uk/index.php?project=NiftyCal
[citation]: http://link.springer.com/article/10.1007%2Fs11548-014-1124-7
[niftk]: http://www.niftk.org
[NiftyCalHome]: https://cmiclab.cs.ucl.ac.uk/CMIC/NiftyCal
[doxygen]: http://cmic.cs.ucl.ac.uk/NiftyCal-API
