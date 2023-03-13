#ifndef __GridMap_h_
#define __GridMap_h_

#include "OccGridMapBase.h"
#include "GridMapLogOdds.h"
#include "GridMapReflectanceCount.h"
#include "GridMapSimpleCount.h"

namespace mapmoverslam {

typedef OccGridMapBase<LogOddsCell, GridMapLogOddsFunctions> GridMap;
//typedef OccGridMapBase<SimpleCountCell, GridMapSimpleCountFunctions> GridMap;
//typedef OccGridMapBase<ReflectanceCell, GridMapReflectanceFunctions> GridMap;

}

#endif
