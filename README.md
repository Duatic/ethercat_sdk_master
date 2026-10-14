# ethercat_sdk_master

[![Humble](https://github.com/Duatic/ethercat_sdk_master/raw/badges/humble.svg)](https://github.com/Duatic/ethercat_sdk_master/actions/workflows/ci.yml)
[![Jazzy](https://github.com/Duatic/ethercat_sdk_master/raw/badges/jazzy.svg)](https://github.com/Duatic/ethercat_sdk_master/actions/workflows/ci.yml)
[![Kilted](https://github.com/Duatic/ethercat_sdk_master/raw/badges/kilted.svg)](https://github.com/Duatic/ethercat_sdk_master/actions/workflows/ci.yml)
[![Lyrical](https://github.com/Duatic/ethercat_sdk_master/raw/badges/lyrical.svg)](https://github.com/Duatic/ethercat_sdk_master/actions/workflows/ci.yml)
[![Rolling](https://github.com/Duatic/ethercat_sdk_master/raw/badges/rolling.svg)](https://github.com/Duatic/ethercat_sdk_master/actions/workflows/ci.yml)

__NOTE:__ This is a fork of the original [repository](https://github.com/leggedrobotics/ethercat_sdk_master) maintained by [Duatic](https://duatic.com)

The 'ethercat_sdk_master' is a common master implementation that can be used to interface multiple ethercat slaves on the same bus.
An example how this package can be found in the 'elmo_ethercat_sdk'

# Dependencies

This list lists all non system or default ROS2 dependencies. For a full list see the [package.xml](./package.xml)

| Repo           | url                                                  | License      | Content                                          |
|:--------------:|:----------------------------------------------------:|:------------:|:------------------------------------------------:|
| soem_interface_rsl |https://github.com/Duatic/soem_interface/ | GPLv3        | EtherCAT functionalities                         |
| duatic_message_logger | https://github.com/Duatic/duatic_message_logger | BSD 3-Clause | simple logging wrapper around ROS2 or cout |
