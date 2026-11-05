^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ecat_sdk_master
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2026-11-05)
------------------
* refactor!: rename the package to ecat_sdk_master (#15)
* feat: share one master per network interface through EthercatMasterSingleton, reference counted
  and set up in two stages through handles
* feat: rework the EtherCAT bus startup (#3)
* feat: support ROS 2 lyrical (#6)
* feat: log the shutdown of each master
* deprecate: EthercatMaster::setRealtimePriority, use realtime_tools instead
* fix: return false when adding a slave fails (#9)
* fix: call the bus startup finished callback only once for the first device
* fix: shut the master down in the right order
* fix: activate the master once its bus has started
* fix: return a result from every path of the singleton
* fix: refuse to release a handle twice, instead of refusing a valid one
* docs: note in the README that this is a fork
