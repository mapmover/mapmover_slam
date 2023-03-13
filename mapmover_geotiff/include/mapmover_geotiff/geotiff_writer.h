#ifndef _GEOTIFFWRITER_H__
#define _GEOTIFFWRITER_H__

#include "map_writer_interface.h"

#include <Eigen/Geometry>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <QImage>
#include <QApplication>
#include <QFont>

#include <mapmover_map_tools/MapmoverMapTools.h>

#if  __cplusplus < 201703L
	#include <experimental/filesystem>
	namespace fs = std::experimental::filesystem;
#else
	#include <filesystem>
	namespace fs = std::filesystem;
#endif


namespace mapmover_geotiff{


class GeotiffWriter : public MapWriterInterface
{
  public:
  explicit GeotiffWriter(bool useCheckerboardCacheIn = false);
  virtual ~GeotiffWriter();

  //setUsePrecalcGrid(bool usePrecalc, const Eigen::Vector2f& size);

  void setMapFileName(const std::string& mapFileName);
  void setMapFilePath(const std::string& mapFilePath);
  void setUseUtcTimeSuffix(bool useSuffix);

  void setupImageSize();
  bool setupTransforms(const nav_msgs::OccupancyGrid& map);
  void drawBackgroundCheckerboard();
  void drawMap(const nav_msgs::OccupancyGrid& map, bool draw_explored_space_grid = true);
  void drawObjectOfInterest(const Eigen::Vector2f& coords, const std::string& txt, const Color& color, const Shape& shape);
  inline virtual void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points){
      drawPath(start, points, 120,0,240);
  }
  void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points, int color_r, int color_g, int color_b);
  void drawCoords();
  std::string getBasePathAndFileName() const;
  void writeGeotiffImage(bool completed);


protected:

  void transformPainterToImgCoords(QPainter& painter);
  void drawCross(QPainter& painter, const Eigen::Vector2f& coords);
  void drawArrow(QPainter& painter);
  void drawCoordSystem(QPainter& painter);

  float resolution = std::numeric_limits<float>::quiet_NaN();
  Eigen::Vector2f origin;

  int resolutionFactor = 3;
  float resolutionFactorf = std::numeric_limits<float>::quiet_NaN();

  bool useCheckerboardCache;
  bool use_utc_time_suffix_;

  float pixelsPerMapMeter = std::numeric_limits<float>::quiet_NaN();
  float pixelsPerGeoTiffMeter = std::numeric_limits<float>::quiet_NaN();

  Eigen::Vector2i minCoordsMap;
  Eigen::Vector2i maxCoordsMap;

  Eigen::Vector2i sizeMap;
  Eigen::Vector2f sizeMapf;

  Eigen::Vector2f rightBottomMarginMeters;
  Eigen::Vector2f rightBottomMarginPixelsf;
  Eigen::Vector2i rightBottomMarginPixels;

  Eigen::Vector2f leftTopMarginMeters;

  Eigen::Vector2f totalMeters;

  Eigen::Vector2i geoTiffSizePixels;

  Eigen::Vector2f mapOrigInGeotiff;
  Eigen::Vector2f mapEndInGeotiff;

  std::string map_file_name_;
  std::string map_file_path_;

  QImage image;
  QImage checkerboard_cache;
  QApplication* app;
  QString font_family_;
  QFont map_draw_font_;

  MapmoverMapTools::CoordinateTransformer<float> world_map_transformer_;
  MapmoverMapTools::CoordinateTransformer<float> map_geo_transformer_;
  MapmoverMapTools::CoordinateTransformer<float> world_geo_transformer_;

  nav_msgs::MapMetaData cached_map_meta_data_;
};

}

#endif
