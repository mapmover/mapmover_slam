#include "mapmover_geotiff/geotiff_writer.h"

#include <cstdio>
#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/GetMap.h>

#include <QApplication>

using namespace std;

namespace mapmover_geotiff{

/**
 * @brief Map generation node.
 */
class MapGenerator
{
  public:
    MapGenerator(const std::string& mapname) : mapname_(mapname)
    {
      ros::NodeHandle n;
      ROS_INFO("Waiting for the map");
      map_sub_ = n.subscribe("map", 1, &MapGenerator::mapCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGridConstPtr& map)
    {
      ros::Time start_time (ros::Time::now());

      geotiff_writer.setMapFileName(mapname_);
      geotiff_writer.setupTransforms(*map);
      geotiff_writer.drawBackgroundCheckerboard();
      geotiff_writer.drawMap(*map);
      geotiff_writer.drawCoords();

      geotiff_writer.writeGeotiffImage(true);

      ros::Duration elapsed_time (ros::Time::now() - start_time);
      ROS_INFO("GeoTiff created in %f seconds", elapsed_time.toSec());
    }

    GeotiffWriter geotiff_writer;

    std::string mapname_;
    ros::Subscriber map_sub_;
};

}

#define USAGE "Usage: \n" \
              "  geotiff_saver -h\n"\
              "  geotiff_saver [-f <mapname>] [ROS remapping args]"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_saver");
  std::string mapname = "map";

  for(int i=1; i<argc; i++)
  {
    if(!strcmp(argv[i], "-h"))
    {
      puts(USAGE);
      return 0;
    }
    else if(!strcmp(argv[i], "-f"))
    {
      if(++i < argc)
        mapname = argv[i];
      else
      {
        puts(USAGE);
        return 1;
      }
    }
    else
    {
      puts(USAGE);
      return 1;
    }
  }

  //GeotiffWriter geotiff_writer;
  //geotiff_writer.setMapName("test");
  mapmover_geotiff::MapGenerator mg(mapname);

  ros::spin();

  return 0;
}

