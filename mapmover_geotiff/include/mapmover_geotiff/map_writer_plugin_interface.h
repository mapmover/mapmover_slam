#ifndef _MAPWRITERPLUGININTERFACE_H__
#define _MAPWRITERPLUGININTERFACE_H__

#include "map_writer_interface.h"

namespace mapmover_geotiff{

class MapWriterPluginInterface{

public:

  virtual void initialize(const std::string& name) = 0;
  virtual void draw(MapWriterInterface* map_writer_interface) = 0;

};

} //namespace mapmover_geotiff

#endif
