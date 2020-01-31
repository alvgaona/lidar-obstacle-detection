#include "environment.h"

#include "process_point_clouds.cpp"
#include "process_point_clouds.h"
#include "render.h"

std::vector<Car> Environment::InitHighway(bool render_scene, pcl::visualization::PCLVisualizer::Ptr& viewer) {
  Render render;

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (render_scene) {
    render.RenderHighway(viewer);
    egoCar.Render(viewer);
    car1.Render(viewer);
    car2.Render(viewer);
    car3.Render(viewer);
  }

  return cars;
}

void Environment::SimpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool render_scene = false;
  std::vector<Car> cars = InitHighway(render_scene, viewer);
  Render render;

  std::unique_ptr<Lidar> lidar(new Lidar(cars, 0.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = lidar->Scan();
  render.RenderRays(viewer, lidar->position, input_cloud);
  render.RenderPointCloud(viewer, input_cloud, "Input Cloud");

  ProcessPointClouds<pcl::PointXYZ> point_processor;
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_result =
      point_processor.SegmentPlane(input_cloud, 100, 0.2);
  render.RenderPointCloud(viewer, segment_result.first, "Obstacle Cloud", Color(1,0,0));
  render.RenderPointCloud(viewer, segment_result.second, "Plane Cloud", Color(0,1,0));

}

// camera_angle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void Environment::InitCamera(CameraAngle camera_angle, pcl::visualization::PCLVisualizer::Ptr& viewer) {
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  int distance = 16;

  switch (camera_angle) {
    case XY:
      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    case TopDown:
      viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    case Side:
      viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    case FPS:
      viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (camera_angle != FPS) {
    viewer->addCoordinateSystem(1.0);
  }
}
