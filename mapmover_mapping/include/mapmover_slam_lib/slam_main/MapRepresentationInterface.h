#ifndef _mapmovermaprepresentationinterface_h__
#define _mapmovermaprepresentationinterface_h__

class GridMap;
class ConcreteOccGridMapUtil;
class DataContainer;

namespace mapmoverslam{

class MapRepresentationInterface
{
public:

  virtual ~MapRepresentationInterface() {};

  virtual void reset() = 0;

  virtual float getScaleToMap() const = 0;

  virtual int getMapLevels() const = 0;
  virtual const GridMap& getGridMap(int mapLevel = 0) const = 0;

  virtual void addMapMutex(int i, MapLockerInterface* mapMutex) = 0;
  virtual MapLockerInterface* getMapMutex(int i) = 0;

  virtual void onMapUpdated() = 0;

  virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix) = 0;

  virtual void updateByScan(const DataContainer& dataContainer, const Eigen::Vector3f& robotPoseWorld) = 0;

  virtual void setUpdateFactorFree(float free_factor) = 0;
  virtual void setUpdateFactorOccupied(float occupied_factor) = 0;
};

}

#endif
