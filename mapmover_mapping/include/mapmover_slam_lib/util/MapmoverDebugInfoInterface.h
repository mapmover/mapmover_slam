#ifndef mapmoverdebuginfointerface_h__
#define mapmoverdebuginfointerface_h__

class MapmoverDebugInfoInterface{
public:

  virtual void sendAndResetData() = 0;
  virtual void addHessianMatrix(const Eigen::Matrix3f& hessian) = 0;
  virtual void addPoseLikelihood(float lh) = 0;
};

#endif
