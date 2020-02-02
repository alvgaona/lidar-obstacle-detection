#ifndef LIDAR_OBSTACLE_DETECTION_ENVIRONMENT_H_
#define LIDAR_OBSTACLE_DETECTION_ENVIRONMENT_H_

#include "lidar.h"
#include "process_point_clouds.h"
#include "render.h"

class Environment {
 public:
  Environment() = default;
  ~Environment() = default;

  std::vector<Car> InitHighway(bool render_scene, pcl::visualization::PCLVisualizer::Ptr& viewer);
  void SimpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer);
  void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer);
  void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& point_processor,
                 const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud);
  void InitCamera(CameraAngle camera_angle, pcl::visualization::PCLVisualizer::Ptr& viewer);
};

#endif