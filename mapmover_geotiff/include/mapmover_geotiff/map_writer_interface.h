#ifndef _MAPWRITERINTERFACE_H__
#define _MAPWRITERINTERFACE_H__

#include <vector>
#include <Eigen/Core>

namespace mapmover_geotiff{

enum Shape {
  SHAPE_CIRCLE,
  SHAPE_DIAMOND
};

class MapWriterInterface{
public:
  struct Color {
    Color(unsigned int r, unsigned int g, unsigned int b) : r(r), g(g), b(b) {}
    unsigned int r,g,b;
  };

  bool completed_map_ = false;

  virtual std::string getBasePathAndFileName() const = 0;
  virtual void drawObjectOfInterest(const Eigen::Vector2f& coords, const std::string& txt, const Color& color, const Shape& shape = SHAPE_CIRCLE) = 0;
  //virtual void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points) = 0;

  inline virtual void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points){
      drawPath(start, points, 120,0,240);
  }
  virtual void drawPath(const Eigen::Vector3f& start, const std::vector<Eigen::Vector2f>& points, int color_r, int color_g, int color_b) = 0;

};

}

#endif
