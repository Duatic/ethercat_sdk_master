1.0.0 (2026-11-05)
------------------
* refactor!: rename to ecat_sdk_master and publish to the public apt archive (#15)
* fix: return false when adding a Slave fails (#9)
* Update readme
* CI & README standardization (#8)
* try to get rolling build working
* Bump actions/cache from 5 to 6 (#5)
* Bump actions/checkout from 6 to 7 (#4)
* added support for lyrical (#6)
* fix issue with bus startup finished callback being called twice for the first device
* added ci
* Rework ethercat bus startup (#3)
* fix bug in shutdown
* added another debug log
* added debug print
* added running flag to handle
* added debug print out
* added missing activate
* fix missing return
* fix singleton implementation
* added an additional logging outpu
* refactor release method to accept a handle
* rework singleton to support a 2 stage setup process
* Rework EthercatMasterSingleton to implement some kind of reference counting
* start to rework the singleton so it can be safely used
* remove Wshadow
* disable old style cast warning
* added note to readme about this being a fork
