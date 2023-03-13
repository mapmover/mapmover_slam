^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mapmover_imu_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2021-04-08)
------------------

0.5.1 (2021-01-15)
------------------

0.5.0 (2020-12-17)
------------------
* Moved mapmover_geotiff launch files to separate package to solve cyclic dependency.
  Clean up for noetic release.
* Bump CMake version to avoid CMP0048 warning
* Contributors: Marius Schnaubelt, Stefan Fabian

0.4.1 (2020-05-15)
------------------

0.3.6 (2019-10-31)
------------------

0.3.5 (2016-06-24)
------------------

0.3.4 (2015-11-07)
------------------
* Fix sim setup
* remap for bertl setup
* Contributors: Florian Kunz, kohlbrecher

0.3.3 (2014-06-15)
------------------
* mapmover_imu_tools: Basics of simple height etimation
* mapmover_imu_tools: Add tf publishers in mapmover_imu_tools
* mapmover_imu_tools: Also write out fake odometry
* mapmover_imu_tools: Fix typo
* mapmover_imu_tools: Prevent race conditions in slam, formatting
* mapmover_imu_tools: Small executable for generating a IMU message out of a (2d) pose and roll/pitch imu message
* Contributors: Stefan Kohlbrecher
