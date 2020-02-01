#include "environment.h"
#include "process_point_clouds.cpp"

int main(int argc, char** argv) {
  std::cout << "Starting enviroment." << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;

  Environment environment;
  environment.InitCamera(setAngle, viewer);
  environment.CityBlock(viewer);

  ProcessPointClouds<pcl::PointXYZI> point_processor;

  std::vector<boost::filesystem::path> stream = point_processor.StreamPcd("resources/data/pcd/data_1");

  auto stream_itr = stream.begin();

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud;

  while (!viewer->wasStopped()) {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    input_cloud = point_processor.LoadPcd(stream_itr->string());
    environment.CityBlock(viewer, point_processor, input_cloud);

    stream_itr++;
    if (stream_itr == stream.end()) {
      stream_itr = stream.begin();
    }
    viewer->spinOnce();
  }
}
