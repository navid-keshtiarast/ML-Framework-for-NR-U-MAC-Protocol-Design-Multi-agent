NR-U Extension Release Notes                         {#nrureleasenotes}
============================

This file contains release notes for the NR-U extension module (most recent releases first).  

All of the ns-3 documentation is accessible from the ns-3 website:
http://www.nsnam.org including tutorials: http://www.nsnam.org/tutorials.html

Consult the file CHANGES.md for more detailed information about changed
API and behavior across releases.


Release NR-U-v0.3
-----------------

Availability
------------
This release is not yet available.

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note:  not all features available on all platforms):

- g++-4.9 or later
- Apple LLVM version 7.0.2 or later
- clang-3.3 or later

In addition, Python 2.7 (Python 2 series) or Python 3.4-3.7 (Python 3 series)

This release has been tested on the following platforms:
- Ubuntu 18.04.5 LTS (64 bit) with g++-7.5.0 and Python 3.6.9

New user-visible features (old first)
-------------------------



Bugs fixed
----------


Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr-u




Release NR-U-v0.2
----------------
In this release, nr-u module is upgraded to be compatible with the latest official ns-3-dev and nr modules.

The upgrade, completed by Rediet (https://gitlab.com/rediet), contains: 
 
- Standard renaming for class/file/variable names (MmWave -> Nr, Enb -> Gnb)
- Deprecated NR classes MmWavePhyMacCommon, BandwidthPartRepresentation replaced with CcBwpCreator, OperationBandInfo, IdealBeamformingHelper
- wscript: added libraries for classes now part of the ns-3 core instead of NR

In future we would like to include the access manager into mainline for nr ( when nr-u .rst doc, working example, and test will be available).


Availability
------------
This release is available from May 28, 2021. Is supports nr module release 5g-lena-v1.1.y 
and later. Please, install first nr module. Clone this module under the src/ directory 
of ns-3-dev (or contrib/ directory of ns-3-dev), as explained in the NR module README.

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note:  not all features available on all platforms):

- g++-4.9 or later
- Apple LLVM version 7.0.2 or later
- clang-3.3 or later

In addition, Python 2.7 (Python 2 series) or Python 3.4-3.7 (Python 3 series)

This release has been tested on the following platforms:
- Ubuntu 18.04.5 LTS (64 bit) with g++-7.5.0 and Python 3.6.9


New user-visible features (old first)
-------------------------
- [none]


Bugs fixed
----------
- [none]


Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr-u


Release NR-U-v0.1
-----------------
In this release, multiple access managers have been implemented. Other classes
are for defining multiple-technology scenarios, in which each technology can be
configured and from which results can be obtained.

Unfortunately, the example we have setup depends on IMDEA WiGig, heavily
modified by us, that we are not allowed to release. Therefore, no examples
can be run.

In the future, we will probably merge the access manager into our mainline for nr.

Availability
------------
This release is available from July 17, 2020. It is based on the v0.4 of NR module.
Please copy this module under the src/ directory of ns-3-dev, as installed as explained
in the NR module README.

Supported platforms
-------------------
This release is intended to work on systems with the following minimal
requirements (Note:  not all features available on all platforms):

- g++-4.9 or later
- Apple LLVM version 7.0.2 or later
- clang-3.3 or later

In addition, Python 2.7 (Python 2 series) or Python 3.4-3.7 (Python 3 series)

This release has been tested on the following platforms:
- ArchLinux 2020.07.01 with g++-10.1.0 and Python 3.8.3
- Ubuntu 16.04 (64 bit) with g++-5.4.0 and Python 2.7.12/3.5.2

New user-visible features (old first)
-------------------------
- Implemented different LBT algorithms

Bugs fixed
----------
- [none]

Known issues
------------
In general, known issues are tracked on the project tracker available
at https://gitlab.com/cttc-lena/nr-u
