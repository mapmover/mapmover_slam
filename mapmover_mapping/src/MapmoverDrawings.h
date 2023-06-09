#ifndef MAPMOVER_DRAWINGS_H__
#define MAPMOVER_DRAWINGS_H__

#include "util/DrawInterface.h"
#include "util/UtilFunctions.h"

#include "ros/ros.h"

#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>


class MapmoverDrawings : public DrawInterface
{
public:

  MapmoverDrawings()
  {
    idCounter = 0;

    ros::NodeHandle nh_;

    markerPublisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    markerArrayPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);

    tempMarker.header.frame_id = "map";
    tempMarker.ns = "slam";

    this->setScale(1.0);
    this->setColor(1.0, 1.0, 1.0);

    tempMarker.action = visualization_msgs::Marker::ADD;
  };

  virtual void drawPoint(const Eigen::Vector2f& pointWorldFrame)
  {
    tempMarker.id = idCounter++;

    tempMarker.pose.position.x = pointWorldFrame.x();
    tempMarker.pose.position.y = pointWorldFrame.y();

    tempMarker.pose.orientation.w = 0.0;
    tempMarker.pose.orientation.z = 0.0;
    tempMarker.type = visualization_msgs::Marker::CUBE;

    //markerPublisher_.publish(tempMarker);

    markerArray.markers.push_back(tempMarker);
  }

  virtual void drawArrow(const Eigen::Vector3f& poseWorld)
  {
    tempMarker.id = idCounter++;

    tempMarker.pose.position.x = poseWorld.x();
    tempMarker.pose.position.y = poseWorld.y();

    tempMarker.pose.orientation.w = cos(poseWorld.z()*0.5f);
    tempMarker.pose.orientation.z = sin(poseWorld.z()*0.5f);

    tempMarker.type = visualization_msgs::Marker::ARROW;

    //markerPublisher_.publish(tempMarker);

    markerArray.markers.push_back(tempMarker);

  }

  virtual void drawCovariance(const Eigen::Vector2f& mean, const Eigen::Matrix2f& covMatrix)
  {
    tempMarker.pose.position.x = mean[0];
    tempMarker.pose.position.y = mean[1];

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covMatrix);

    const Eigen::Vector2f& eigValues (eig.eigenvalues());
    const Eigen::Matrix2f& eigVectors (eig.eigenvectors());

    float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

    tempMarker.type = visualization_msgs::Marker::CYLINDER;

    double lengthMajor = sqrt(eigValues[0]);
    double lengthMinor = sqrt(eigValues[1]);

    tempMarker.scale.x = lengthMajor;
    tempMarker.scale.y = lengthMinor;
    tempMarker.scale.z = 0.001;


    tempMarker.pose.orientation.w = cos(angle*0.5);
    tempMarker.pose.orientation.z = sin(angle*0.5);

    tempMarker.id = idCounter++;
    markerArray.markers.push_back(tempMarker);

    //drawLine(Eigen::Vector3f(0,0,0), Eigen::Vector3f(lengthMajor ,0,0));
    //drawLine(Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,lengthMinor,0));

    //glScalef(lengthMajor, lengthMinor, 0);
    //glCallList(dlCircle);
    //this->popCS();
  }

  virtual void setScale(double scale)
  {
    tempMarker.scale.x = scale;
    tempMarker.scale.y = scale;
    tempMarker.scale.z = scale;
  }

  virtual void setColor(double r, double g, double b, double a = 1.0)
  {
    tempMarker.color.r = r;
    tempMarker.color.g = g;
    tempMarker.color.b = b;
    tempMarker.color.a = a;
  }

  virtual void sendAndResetData()
  {
    markerArrayPublisher_.publish(markerArray);
    markerArray.markers.clear();
    idCounter = 0;
  }

  void setTime(const ros::Time& time)
  {
    tempMarker.header.stamp = time;
  }


  ros::Publisher markerPublisher_;
  ros::Publisher markerArrayPublisher_;

  visualization_msgs::Marker tempMarker;
  visualization_msgs::MarkerArray markerArray;

  int idCounter;
};

#endif
