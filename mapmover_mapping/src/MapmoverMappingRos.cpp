#include "MapmoverMappingRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "sensor_msgs/PointCloud2.h"

#include "MapmoverDrawings.h"
#include "MapmoverDebugInfoProvider.h"
#include "MapmoverMapMutex.h"

#ifndef TF_SCALAR_H
  typedef btScalar tfScalar;
#endif

MapmoverMappingRos::MapmoverMappingRos()
  : debugInfoProvider(0)
  , mapmoverDrawings(0)
  , lastGetMapUpdateIndex(-100)
  , tfB_(0)
  , map__publish_thread_(0)
  , initial_pose_set_(false)
  , pause_scan_processing_(false)
{
  ros::NodeHandle private_nh_("~");

  std::string mapTopic_ = "map";

  private_nh_.param("pub_drawings", p_pub_drawings, false);
  private_nh_.param("pub_debug_output", p_pub_debug_output_, false);
  private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_,true);
  private_nh_.param("pub_odometry", p_pub_odometry_,false);
  private_nh_.param("advertise_map_service", p_advertise_map_service_,true);
  private_nh_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);

  private_nh_.param("map_resolution", p_map_resolution_, 0.025);
  private_nh_.param("map_size", p_map_size_, 1024);
  private_nh_.param("map_start_x", p_map_start_x_, 0.5);
  private_nh_.param("map_start_y", p_map_start_y_, 0.5);
  private_nh_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

  private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
  private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

  private_nh_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
  private_nh_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

  private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
  private_nh_.param("sys_msg_topic", p_sys_msg_topic_, std::string("syscommand"));
  private_nh_.param("pose_update_topic", p_pose_update_topic_, std::string("poseupdate"));

  private_nh_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_,true);
  private_nh_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_,false);
  private_nh_.param("map_with_known_poses", p_map_with_known_poses_, false);

  private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
  private_nh_.param("map_frame", p_map_frame_, std::string("map"));
  private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));

  private_nh_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_,true);
  private_nh_.param("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_, std::string("scanmatcher_frame"));

  private_nh_.param("output_timing", p_timing_output_,false);

  private_nh_.param("map_pub_period", p_map_pub_period_, 2.0);

  double tmp = 0.0;
  private_nh_.param("laser_min_dist", tmp, 0.4);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_max_dist", tmp, 30.0);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_z_min_value", tmp, -1.0);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  private_nh_.param("laser_z_max_value", tmp, 1.0);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    ROS_INFO("MapmoverSM publishing debug drawings");
    mapmoverDrawings = new MapmoverDrawings();
  }

  if(p_pub_debug_output_)
  {
    ROS_INFO("MapmoverSM publishing debug info");
    debugInfoProvider = new MapmoverDebugInfoProvider();
  }

  if(p_pub_odometry_)
  {
    odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("scanmatch_odom", 50);
  }

  slamProcessor = new mapmoverslam::MapmoverSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, mapmoverDrawings, debugInfoProvider);
  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new MapmoverMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
    tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

    if ( (i == 0) && p_advertise_map_service_)
    {
      tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &MapmoverMappingRos::mapCallback, this);
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if ( i== 0){
      mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
    }
  }

  // Initialize services
  reset_map_service_ = node_.advertiseService("reset_map", &MapmoverMappingRos::resetMapCallback, this);
  restart_mapmover_service_ = node_.advertiseService("restart_mapping_with_new_pose", &MapmoverMappingRos::restartMapmoverCallback, this);
  toggle_scan_processing_service_ = node_.advertiseService("pause_mapping", &MapmoverMappingRos::pauseMapCallback, this);

  ROS_INFO("MapmoverSM p_base_frame_: %s", p_base_frame_.c_str());
  ROS_INFO("MapmoverSM p_map_frame_: %s", p_map_frame_.c_str());
  ROS_INFO("MapmoverSM p_odom_frame_: %s", p_odom_frame_.c_str());
  ROS_INFO("MapmoverSM p_scan_topic_: %s", p_scan_topic_.c_str());
  ROS_INFO("MapmoverSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  ROS_INFO("MapmoverSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  ROS_INFO("MapmoverSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  ROS_INFO("MapmoverSM p_map_pub_period_: %f", p_map_pub_period_);
  ROS_INFO("MapmoverSM p_update_factor_free_: %f", p_update_factor_free_);
  ROS_INFO("MapmoverSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  ROS_INFO("MapmoverSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  ROS_INFO("MapmoverSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  ROS_INFO("MapmoverSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  ROS_INFO("MapmoverSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  scanSubscriber_ = node_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &MapmoverMappingRos::scanCallback, this);
  sysMsgSubscriber_ = node_.subscribe(p_sys_msg_topic_, 2, &MapmoverMappingRos::sysMsgCallback, this);

  poseUpdatePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_update_topic_, 1, false);
  posePublisher_ = node_.advertise<geometry_msgs::PoseStamped>("slam_out_pose", 1, false);

  scan_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud>("slam_cloud",1,false);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  /*
  bool p_use_static_map_ = false;

  if (p_use_static_map_){
    mapSubscriber_ = node_.subscribe(mapTopic_, 1, &MapmoverMappingRos::staticMapCallback, this);
  }
  */

  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(node_, "initialpose", 2);
  initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, tf_, p_map_frame_, 2);
  initial_pose_filter_->registerCallback(boost::bind(&MapmoverMappingRos::initialPoseCallback, this, _1));


  map__publish_thread_ = new boost::thread(boost::bind(&MapmoverMappingRos::publishMapLoop, this, p_map_pub_period_));

  map_to_odom_.setIdentity();

  lastMapPublishTime = ros::Time(0,0);
}

MapmoverMappingRos::~MapmoverMappingRos()
{
  delete slamProcessor;

  if (mapmoverDrawings)
    delete mapmoverDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;

  if (tfB_)
    delete tfB_;

  if(map__publish_thread_)
    delete map__publish_thread_;
}

void MapmoverMappingRos::scanCallback(const sensor_msgs::LaserScan& scan)
{
  if (pause_scan_processing_)
  {
    return;
  }

  if (mapmoverDrawings)
  {
    mapmoverDrawings->setTime(scan.header.stamp);
  }

  ros::WallTime start_time = ros::WallTime::now();
  if (!p_use_tf_scan_transformation_)
  {
    // If we are not using the tf tree to find the transform between the base frame and laser frame,
    // then just convert the laser scan to our data container and process the update based on our last
    // pose estimate
    this->rosLaserScanToDataContainer(scan, laserScanContainer, slamProcessor->getScaleToMap());
    slamProcessor->update(laserScanContainer, slamProcessor->getLastScanMatchPose());
  }
  else
  {
    // If we are using the tf tree to find the transform between the base frame and laser frame,
    // let's get that transform
    const ros::Duration dur(0.5);
    tf::StampedTransform laser_transform;
    if (tf_.waitForTransform(p_base_frame_, scan.header.frame_id, scan.header.stamp, dur))
    {
      tf_.lookupTransform(p_base_frame_, scan.header.frame_id, scan.header.stamp, laser_transform);
    }
    else
    {
      ROS_INFO("lookupTransform %s to %s timed out. Could not transform laser scan into base_frame.", p_base_frame_.c_str(), scan.header.frame_id.c_str());
      return;
    }

    // Convert the laser scan to point cloud
    projector_.projectLaser(scan, laser_point_cloud_, 30.0);

    // Publish the point cloud if there are any subscribers
    if (scan_point_cloud_publisher_.getNumSubscribers() > 0)
    {
      scan_point_cloud_publisher_.publish(laser_point_cloud_);
    }

    // Convert the point cloud to our data container
    this->rosPointCloudToDataContainer(laser_point_cloud_, laser_transform, laserScanContainer, slamProcessor->getScaleToMap());

    // Now let's choose the initial pose estimate for our slam process update
    Eigen::Vector3f start_estimate(Eigen::Vector3f::Zero());
    if (initial_pose_set_)
    {
      // User has requested a pose reset
      initial_pose_set_ = false;
      start_estimate = initial_pose_;
    }
    else if (p_use_tf_pose_start_estimate_)
    {
      // Initial pose estimate comes from the tf tree
      try
      {
        tf::StampedTransform stamped_pose;

        tf_.waitForTransform(p_map_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
        tf_.lookupTransform(p_map_frame_, p_base_frame_,  scan.header.stamp, stamped_pose);

        const double yaw = tf::getYaw(stamped_pose.getRotation());
        start_estimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(), stamped_pose.getOrigin().getY(), yaw);
      }
      catch(tf::TransformException e)
      {
        ROS_ERROR("Transform from %s to %s failed\n", p_map_frame_.c_str(), p_base_frame_.c_str());
        start_estimate = slamProcessor->getLastScanMatchPose();
      }
    }
    else
    {
      // If none of the above, the initial pose is simply the last estimated pose
      start_estimate = slamProcessor->getLastScanMatchPose();
    }

    // If "p_map_with_known_poses_" is enabled, we assume that start_estimate is precise and doesn't need to be refined
    if (p_map_with_known_poses_)
    {
      slamProcessor->update(laserScanContainer, start_estimate, true);
    }
    else
    {
      slamProcessor->update(laserScanContainer, start_estimate);
    }
  }

  // If the debug flag "p_timing_output_" is enabled, print how long this last iteration took
  if (p_timing_output_)
  {
    ros::WallDuration duration = ros::WallTime::now() - start_time;
    ROS_INFO("MapmoverSLAM Iter took: %f milliseconds", duration.toSec()*1000.0f );
  }

  // If we're just building a map with known poses, we're finished now. Code below this point publishes the localization results.
  if (p_map_with_known_poses_)
  {
    return;
  }

  poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan.header.stamp, p_map_frame_);

  // Publish pose with and without covariances
  poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());
  posePublisher_.publish(poseInfoContainer_.getPoseStamped());

  // Publish odometry if enabled
  if(p_pub_odometry_)
  {
    nav_msgs::Odometry tmp;
    tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

    tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
    tmp.child_frame_id = p_base_frame_;
    odometryPublisher_.publish(tmp);
  }

  // Publish the map->odom transform if enabled
  if (p_pub_map_odom_transform_)
  {
    tf::StampedTransform odom_to_base;
    try
    {
      tf_.waitForTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
      tf_.lookupTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, odom_to_base);
    }
    catch(tf::TransformException e)
    {
      ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
      odom_to_base.setIdentity();
    }
    map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
    tfB_->sendTransform( tf::StampedTransform (map_to_odom_, scan.header.stamp, p_map_frame_, p_odom_frame_));
  }

  // Publish the transform from map to estimated pose (if enabled)
  if (p_pub_map_scanmatch_transform_)
  {
    tfB_->sendTransform( tf::StampedTransform(poseInfoContainer_.getTfTransform(), scan.header.stamp, p_map_frame_, p_tf_map_scanmatch_transform_frame_name_));
  }
}

void MapmoverMappingRos::sysMsgCallback(const std_msgs::String& string)
{
  ROS_INFO("MapmoverSM sysMsgCallback, msg contents: %s", string.data.c_str());

  if (string.data == "reset")
  {
    ROS_INFO("MapmoverSM reset");
    slamProcessor->reset();
  }
}

bool MapmoverMappingRos::mapCallback(nav_msgs::GetMap::Request  &req,
                                   nav_msgs::GetMap::Response &res)
{
  ROS_INFO("MapmoverSM Map service called");
  res = mapPubContainer[0].map_;
  return true;
}

bool MapmoverMappingRos::resetMapCallback(std_srvs::Trigger::Request  &req,
                                        std_srvs::Trigger::Response &res)
{
  ROS_INFO("MapmoverSM Reset map service called");
  slamProcessor->reset();
  return true;
}

bool MapmoverMappingRos::restartMapmoverCallback(mapmover_mapping::ResetMapping::Request  &req,
                                             mapmover_mapping::ResetMapping::Response &res)
{
  // Reset map
  ROS_INFO("MapmoverSM Reset map");
  slamProcessor->reset();

  // Reset pose
  this->resetPose(req.initial_pose);

  // Unpause node (in case it is paused)
  this->toggleMappingPause(false);

  // Return success
  return true;
}

bool MapmoverMappingRos::pauseMapCallback(std_srvs::SetBool::Request  &req,
                                        std_srvs::SetBool::Response &res)
{
  this->toggleMappingPause(req.data);
  res.success = true;
  return true;
}

void MapmoverMappingRos::publishMap(MapPublisherContainer& mapPublisher, const mapmoverslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex)
{
  nav_msgs::GetMap::Response& map_ (mapPublisher.map_);

  //only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    for(int i=0; i < size; ++i)
    {
      if(gridMap.isFree(i))
      {
        data[i] = 0;
      }
      else if (gridMap.isOccupied(i))
      {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  mapPublisher.mapPublisher_.publish(map_.map);
}

void MapmoverMappingRos::rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, mapmoverslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }
}

void MapmoverMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, mapmoverslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = pointCloud.points.size();
  //ROS_INFO("size: %d", size);

  dataContainer.clear();

  tf::Vector3 laserPos (laserTransform.getOrigin());
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {

    const geometry_msgs::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

    if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

      if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
        continue;
      }

      tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
      }
    }
  }
}

void MapmoverMappingRos::setServiceGetMapData(nav_msgs::GetMap::Response& map_, const mapmoverslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength()*0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/*
void MapmoverMappingRos::setStaticMapData(const nav_msgs::OccupancyGrid& map)
{
  float cell_length = map.info.resolution;
  Eigen::Vector2f mapOrigin (map.info.origin.position.x + cell_length*0.5f,
                             map.info.origin.position.y + cell_length*0.5f);

  int map_size_x = map.info.width;
  int map_size_y = map.info.height;

  slamProcessor = new mapmoverslam::MapmoverSlamProcessor(cell_length, map_size_x, map_size_y, Eigen::Vector2f(0.0f, 0.0f), 1, mapmoverDrawings, debugInfoProvider);
}
*/


void MapmoverMappingRos::publishMapLoop(double map_pub_period)
{
  ros::Rate r(1.0 / map_pub_period);
  while(ros::ok())
  {
    //ros::WallTime t1 = ros::WallTime::now();
    ros::Time mapTime (ros::Time::now());
    //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
    //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
    publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    //ros::WallDuration t2 = ros::WallTime::now() - t1;

    //std::cout << "time s: " << t2.toSec();
    //ROS_INFO("MapmoverSM ms: %4.2f", t2.toSec()*1000.0f);

    r.sleep();
  }
}

void MapmoverMappingRos::staticMapCallback(const nav_msgs::OccupancyGrid& map)
{

}

void MapmoverMappingRos::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  this->resetPose(msg->pose.pose);
}

void MapmoverMappingRos::toggleMappingPause(bool pause)
{
  // Pause/unpause
  if (pause && !pause_scan_processing_)
  {
    ROS_INFO("[MapmoverSM]: Mapping paused");
  }
  else if (!pause && pause_scan_processing_)
  {
    ROS_INFO("[MapmoverSM]: Mapping no longer paused");
  }
  pause_scan_processing_ = pause;
}

void MapmoverMappingRos::resetPose(const geometry_msgs::Pose &pose)
{
  initial_pose_set_ = true;
  initial_pose_ = Eigen::Vector3f(pose.position.x, pose.position.y, util::getYawFromQuat(pose.orientation));
  ROS_INFO("[MapmoverSM]: Setting initial pose with world coords x: %f y: %f yaw: %f",
           initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}
