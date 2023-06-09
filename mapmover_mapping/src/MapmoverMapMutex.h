#ifndef mapmovermapmutex_h__
#define mapmovermapmutex_h__

#include "util/MapLockerInterface.h"

#include <boost/thread/mutex.hpp>

class MapmoverMapMutex : public MapLockerInterface
{
public:
  virtual void lockMap()
  {
    mapModifyMutex_.lock();
  }

  virtual void unlockMap()
  {
    mapModifyMutex_.unlock();
  }

  boost::mutex mapModifyMutex_;
};

#endif
