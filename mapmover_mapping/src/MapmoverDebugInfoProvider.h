#ifndef MAPMOVER_DEBUG_INFO_PROVIDER_H__
#define MAPMOVER_DEBUG_INFO_PROVIDER_H__

#include "util/MapmoverDebugInfoInterface.h"
#include "util/UtilFunctions.h"

#include "ros/ros.h"

#include "mapmover_mapping/MapmoverDebugInfo.h"


class MapmoverDebugInfoProvider : public MapmoverDebugInfoInterface
{
public:

  MapmoverDebugInfoProvider()
  {
    ros::NodeHandle nh_;

    debugInfoPublisher_ = nh_.advertise<mapmover_mapping::MapmoverDebugInfo>("mapmover_debug_info", 50, true);
  };

  virtual void sendAndResetData()
  {
    debugInfoPublisher_.publish(debugInfo);
    debugInfo.iterData.clear();
  }


  virtual void addHessianMatrix(const Eigen::Matrix3f& hessian)
  {
    mapmover_mapping::MapmoverIterData iterData;

    for (int i=0; i < 9; ++i){
      iterData.hessian[i] = static_cast<double>(hessian.data()[i]);
      iterData.determinant = hessian.determinant();

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(hessian);

      const Eigen::Vector3f& eigValues (eig.eigenvalues());
      iterData.conditionNum = eigValues[2] / eigValues[0];


      iterData.determinant2d = hessian.block<2,2>(0,0).determinant();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig2d(hessian.block<2,2>(0,0));

      const Eigen::Vector2f& eigValues2d (eig2d.eigenvalues());
      iterData.conditionNum2d = eigValues2d[1] / eigValues2d[0];
    }

    debugInfo.iterData.push_back(iterData);
  }

  virtual void addPoseLikelihood(float lh)
  {

  }


  mapmover_mapping::MapmoverDebugInfo debugInfo;

  ros::Publisher debugInfoPublisher_;

};

#endif
