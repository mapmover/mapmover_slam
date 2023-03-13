#ifndef _mapmovermaprepmultimap_h__
#define _mapmovermaprepmultimap_h__

#include "MapRepresentationInterface.h"
#include "MapProcContainer.h"

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"

#include "../util/DrawInterface.h"
#include "../util/MapmoverDebugInfoInterface.h"

namespace mapmoverslam{

class MapRepMultiMap : public MapRepresentationInterface
{

public:
  MapRepMultiMap(float mapResolution, int mapSizeX, int mapSizeY, unsigned int numDepth, const Eigen::Vector2f& startCoords, DrawInterface* drawInterfaceIn, MapmoverDebugInfoInterface* debugInterfaceIn)
  {
    //unsigned int numDepth = 3;
    Eigen::Vector2i resolution(mapSizeX, mapSizeY);

    float totalMapSizeX = mapResolution * static_cast<float>(mapSizeX);
    float mid_offset_x = totalMapSizeX * startCoords.x();

    float totalMapSizeY = mapResolution * static_cast<float>(mapSizeY);
    float mid_offset_y = totalMapSizeY * startCoords.y();

    for (unsigned int i = 0; i < numDepth; ++i){
      std::cout << "MapmoverSM map lvl " << i << ": cellLength: " << mapResolution << " res x:" << resolution.x() << " res y: " << resolution.y() << "\n";
      GridMap* gridMap = new mapmoverslam::GridMap(mapResolution,resolution, Eigen::Vector2f(mid_offset_x, mid_offset_y));
      OccGridMapUtilConfig<GridMap>* gridMapUtil = new OccGridMapUtilConfig<GridMap>(gridMap);
      ScanMatcher<OccGridMapUtilConfig<GridMap> >* scanMatcher = new mapmoverslam::ScanMatcher<OccGridMapUtilConfig<GridMap> >(drawInterfaceIn, debugInterfaceIn);

      mapContainer.push_back(MapProcContainer(gridMap, gridMapUtil, scanMatcher));

      resolution /= 2;
      mapResolution*=2.0f;
    }

    dataContainers.resize(numDepth-1);
  }

  virtual ~MapRepMultiMap()
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      mapContainer[i].cleanup();
    }
  }

  virtual void reset()
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      mapContainer[i].reset();
    }
  }

  virtual float getScaleToMap() const { return mapContainer[0].getScaleToMap(); };

  virtual int getMapLevels() const { return mapContainer.size(); };
  virtual const GridMap& getGridMap(int mapLevel) const { return mapContainer[mapLevel].getGridMap(); };

  virtual void addMapMutex(int i, MapLockerInterface* mapMutex)
  {
    mapContainer[i].addMapMutex(mapMutex);
  }

  MapLockerInterface* getMapMutex(int i)
  {
    return mapContainer[i].getMapMutex();
  }

  virtual void onMapUpdated()
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      mapContainer[i].resetCachedData();
    }
  }

  virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix)
  {
    size_t size = mapContainer.size();

    Eigen::Vector3f tmp(beginEstimateWorld);

    for (int index = size - 1; index >= 0; --index){
      //std::cout << " m " << i;
      if (index == 0){
        tmp  = (mapContainer[index].matchData(tmp, dataContainer, covMatrix, 5));
      }else{
        dataContainers[index-1].setFrom(dataContainer, static_cast<float>(1.0 / pow(2.0, static_cast<double>(index))));
        tmp  = (mapContainer[index].matchData(tmp, dataContainers[index-1], covMatrix, 3));
      }
    }
    return tmp;
  }

  virtual void updateByScan(const DataContainer& dataContainer, const Eigen::Vector3f& robotPoseWorld)
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      //std::cout << " u " <<  i;
      if (i==0){
        mapContainer[i].updateByScan(dataContainer, robotPoseWorld);
      }else{
        mapContainer[i].updateByScan(dataContainers[i-1], robotPoseWorld);
      }
    }
    //std::cout << "\n";
  }

  virtual void setUpdateFactorFree(float free_factor)
  {
    size_t size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      GridMap& map = mapContainer[i].getGridMap();
      map.setUpdateFreeFactor(free_factor);
    }
  }

  virtual void setUpdateFactorOccupied(float occupied_factor)
  {
    size_t size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      GridMap& map = mapContainer[i].getGridMap();
      map.setUpdateOccupiedFactor(occupied_factor);
    }
  }

protected:
  std::vector<MapProcContainer> mapContainer;
  std::vector<DataContainer> dataContainers;
};

}

#endif
