#include "render.h"
#include "lidar.h"
#include "process_point_clouds.h"

class Environment {
 public:
  Environment() = default;
  ~Environment() = default;

  std::vector<Car> InitHighway(bool render_scene, pcl::visualization::PCLVisualizer::Ptr& viewer);
  void SimpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer);
  void InitCamera(CameraAngle camera_angle, pcl::visualization::PCLVisualizer::Ptr& viewer);
};


