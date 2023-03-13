#ifndef maplockerinterface_h__
#define maplockerinterface_h__

class MapLockerInterface
{
public:
  virtual void lockMap() = 0;
  virtual void unlockMap() = 0;
};

#endif
