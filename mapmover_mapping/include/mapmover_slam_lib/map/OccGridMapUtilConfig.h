#ifndef __OccGridMapUtilConfig_h_
#define __OccGridMapUtilConfig_h_

#include "OccGridMapUtil.h"

//#define SLAM_USE_HASH_CACHING
#ifdef SLAM_USE_HASH_CACHING
#include "GridMapCacheHash.h"
typedef GridMapCacheHash GridMapCacheMethod;
#else
#include "GridMapCacheArray.h"
typedef GridMapCacheArray GridMapCacheMethod;
#endif

namespace mapmoverslam {

template<typename ConcreteOccGridMap>
class OccGridMapUtilConfig
  : public OccGridMapUtil<ConcreteOccGridMap, GridMapCacheMethod>
{
public:

  OccGridMapUtilConfig(ConcreteOccGridMap* gridMap = 0)
    : OccGridMapUtil<ConcreteOccGridMap, GridMapCacheMethod>(gridMap)
  {}
};

}

#endif
