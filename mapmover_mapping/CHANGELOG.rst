^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mapmover_mapping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.2 (2021-04-08)
------------------
* Remove tf_conversions as a dependency 
* Add reset mapping (and pose) service 
  * add service to reset the mapping with a new initial pose
* Reorganize scanCallback and add comments 
  * Refactor, reformat and comment scanCallback
  * make rosPointCloudToDataContainer void
  * make rosLaserScanToDataContainer void
* Add pause and reset services to mapmover_mapping 
  * Add pause and reset services to mapmover
* Contributors: Marcelino Almeida

0.5.1 (2021-01-15)
------------------

0.5.0 (2020-12-17)
------------------
* Moved mapmover_geotiff launch files to separate package to solve cyclic dependency.
  Clean up for noetic release.
* Bump CMake version to avoid CMP0048 warning
* fixed compilation under noetic
* Contributors: Marius Schnaubelt, Stefan Fabian

0.4.1 (2020-05-15)
------------------
* Remove unnecessary boost signals find_package
  With Boost >1.69 mapmover_mapping won't build. Furthermore, mapmover_mapping doesn't use signals anywhere.
* Contributors: Sam Pfeiffer

0.3.6 (2019-10-31)
------------------
* Merge pull request from davidbsp/catkin
  populate child_frame_id in odometry msg
* Added child_frame_id in mapmover mapping's odometry msg
* Contributors: David Portugal, Johannes Meyer

0.3.5 (2016-06-24)
------------------
* Use the FindEigen3.cmake module provided by Eigen
* Contributors: Johannes Meyer

0.3.4 (2015-11-07)
------------------

0.3.3 (2014-06-15)
------------------

0.3.2 (2014-03-30)
------------------

0.3.1 (2013-10-09)
------------------
* respect ``p_map_frame_``
* added changelogs

0.3.0 (2013-08-08)
------------------
* catkinized mapmover_slam
* mapmover_mapping: fixed multi-resolution map scan matching index bug in MapRepMultiMap.h
