#include <mapmover_geotiff/map_writer_interface.h>
#include <mapmover_geotiff/map_writer_plugin_interface.h>

#include <ros/ros.h>
#include <mapmover_nav_msgs/GetRobotTrajectory.h>

#include <fstream>

namespace mapmover_geotiff_plugins
{

using namespace mapmover_geotiff;

class TrajectoryMapWriter : public MapWriterPluginInterface
{
public:
  TrajectoryMapWriter();
  virtual ~TrajectoryMapWriter();

  virtual void initialize(const std::string& name);
  virtual void draw(MapWriterInterface *interface);

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient service_client_;

  bool initialized_;
  std::string name_;
  bool draw_all_objects_;
  std::string class_id_;
  int path_color_r_;
  int path_color_g_;
  int path_color_b_;
};

TrajectoryMapWriter::TrajectoryMapWriter()
    : initialized_(false)
{}

TrajectoryMapWriter::~TrajectoryMapWriter()
{}

void TrajectoryMapWriter::initialize(const std::string& name)
{
  ros::NodeHandle plugin_nh("~/" + name);
  std::string service_name_;


  plugin_nh.param("service_name", service_name_, std::string("trajectory"));
  plugin_nh.param("path_color_r", path_color_r_, 120);
  plugin_nh.param("path_color_g", path_color_g_, 0);
  plugin_nh.param("path_color_b", path_color_b_, 240);

  service_client_ = nh_.serviceClient<mapmover_nav_msgs::GetRobotTrajectory>(service_name_);

  initialized_ = true;
  this->name_ = name;
  ROS_INFO_NAMED(name_, "Successfully initialized mapmover_geotiff MapWriter plugin %s.", name_.c_str());
}

void TrajectoryMapWriter::draw(MapWriterInterface *interface)
{
    if(!initialized_) return;

    mapmover_nav_msgs::GetRobotTrajectory srv_path;
    if (!service_client_.call(srv_path)) {
      ROS_ERROR_NAMED(name_, "Cannot draw trajectory, service %s failed", service_client_.getService().c_str());
      return;
    }

    std::vector<geometry_msgs::PoseStamped>& traj_vector (srv_path.response.trajectory.poses);

    size_t size = traj_vector.size();

    std::vector<Eigen::Vector2f> pointVec;
    pointVec.resize(size);

    for (size_t i = 0; i < size; ++i){
      const geometry_msgs::PoseStamped& pose (traj_vector[i]);

      pointVec[i] = Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y);
    }

    if (size > 0){
      //Eigen::Vector3f startVec(pose_vector[0].x,pose_vector[0].y,pose_vector[0].z);
      Eigen::Vector3f startVec(pointVec[0].x(),pointVec[0].y(),0.0f);
      interface->drawPath(startVec, pointVec, path_color_r_, path_color_g_, path_color_b_);
    }
}

} // namespace

//register this planner as a MapWriterPluginInterface plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mapmover_geotiff_plugins::TrajectoryMapWriter, mapmover_geotiff::MapWriterPluginInterface)
