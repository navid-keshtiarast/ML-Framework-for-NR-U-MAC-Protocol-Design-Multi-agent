5G-LENA NR-U Extension Changes       {#nruchanges}
==============================

NR-U module: API and model change history
---------------------------------------

<!-- This ChangeLog is updated in the reverse order with the most recent changes coming first.  Date format:  DD-MM-YYYY -->

ns-3 is an evolving system and there will be API or behavioral changes
from time to time.   Users who try to use scripts or models across
versions of ns-3 may encounter problems at compile time, run time, or
may see the simulation output change.

We have adopted the development policy that we are going to try to ease
the impact of these changes on users by documenting these changes in a
single place (this file), and not by providing a temporary or permanent
backward-compatibility software layer.

A related file is the RELEASE_NOTES.md file in the top level directory.
This file complements RELEASE_NOTES.md by focusing on API and behavioral
changes that users upgrading from one release to the next may encounter.
RELEASE_NOTES attempts to comprehensively list all of the changes
that were made.  There is generally some overlap in the information
contained in RELEASE_NOTES.md and this file.

The goal is that users who encounter a problem when trying to use older
code with newer code should be able to consult this file to find
guidance as to how to fix the problem.  For instance, if a method name
or signature has changed, it should be stated what the new replacement
name is.

Note that users who upgrade the simulator across versions, or who work
directly out of the development tree, may find that simulation output
changes even when the compilation doesn't break, such as when a
simulator default value is changed.  Therefore, it is good practice for
_anyone_ using code across multiple ns-3 releases to consult this file,
as well as the RELEASE_NOTES.md, to understand what has changed over time.

This file is a best-effort approach to solving this issue; we will do
our best but can guarantee that there will be things that fall through
the cracks, unfortunately.  If you, as a user, can suggest improvements
to this file based on your experience, please contribute a patch or drop
us a note on ns-developers mailing list.

---


## Changes from NR-v0 to NR-v0.2

### New API:
-[none]

### Changes to existing API:
Renamed to be compatible with the latest nr module.

### Changed behavior:
-[none]



## Changes from NR-v0 to NR-v0.1

### New API:


### Changes to existing API:


### Changed behavior:
