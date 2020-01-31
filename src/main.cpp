#include "environment.h"

int main(int argc, char** argv) {
  std::cout << "Starting enviroment." << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;

  Environment environment;
  environment.InitCamera(setAngle, viewer);
  environment.SimpleHighway(viewer);

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
