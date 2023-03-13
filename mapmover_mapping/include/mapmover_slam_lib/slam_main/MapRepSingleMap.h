#ifndef _mapmovermaprepsinglemap_h__
#define _mapmovermaprepsinglemap_h__

#include "MapRepresentationInterface.h"

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"

#include "../util/DrawInterface.h"
#include "../util/MapmoverDebugInfoInterface.h"

namespace mapmoverslam{

class MapRepSingleMap : public MapRepresentationInterface
{

public:
  MapRepSingleMap(float mapResolution, DrawInterface* drawInterfaceIn, MapmoverDebugInfoInterface* debugInterfaceIn)
  {
    gridMap = new mapmoverslam::GridMap(mapResolution,Eigen::Vector2i(1024,1024), Eigen::Vector2f(20.0f, 20.0f));
    gridMapUtil = new OccGridMapUtilConfig<GridMap>(gridMap);
    scanMatcher = new mapmoverslam::ScanMatcher<OccGridMapUtilConfig<GridMap> >(drawInterfaceIn, debugInterfaceIn);
  }

  virtual ~MapRepSingleMap()
  {
    delete gridMap;
    delete gridMapUtil;
    delete scanMatcher;
  }

  virtual void reset()
  {
    gridMap->reset();
    gridMapUtil->resetCachedData();
  }

  virtual float getScaleToMap() const { return gridMap->getScaleToMap(); };

  virtual int getMapLevels() const { return 1; };
  virtual const GridMap& getGridMap(int mapLevel) const { return *gridMap; };

  virtual void onMapUpdated()
  {
    gridMapUtil->resetCachedData();
  }

  virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix)
  {
    return scanMatcher->matchData(beginEstimateWorld, *gridMapUtil, dataContainer, covMatrix, 20);
  }

  virtual void updateByScan(const DataContainer& dataContainer, const Eigen::Vector3f& robotPoseWorld)
  {
    gridMap->updateByScan(dataContainer, robotPoseWorld);
  }

protected:
  GridMap* gridMap;
  OccGridMapUtilConfig<GridMap>* gridMapUtil;
  ScanMatcher<OccGridMapUtilConfig<GridMap> >* scanMatcher;
};

}

#endif


