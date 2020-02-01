#include "environment.h"

#include "process_point_clouds.cpp"

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
  // RENDER OPTIONS
  bool render_scene = false;
  std::vector<Car> cars = InitHighway(render_scene, viewer);
  Render render;

  std::unique_ptr<Lidar> lidar(new Lidar(cars, 0.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud = lidar->Scan();
  //  render.RenderRays(viewer, lidar->position, input_cloud);
  //  render.RenderPointCloud(viewer, input_cloud, "Input Cloud");

  ProcessPointClouds<pcl::PointXYZ> point_processor;
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_cloud =
      point_processor.CustomRansacSegmentPlane(input_cloud, 100, 0.2);
  render.RenderPointCloud(viewer, segment_cloud.first, "Obstacle Cloud", Color(1, 0, 0));
  render.RenderPointCloud(viewer, segment_cloud.second, "Plane Cloud", Color(0, 1, 0));

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters = point_processor.CustomClustering(segment_cloud.first, 1.0, 3, 30);

  int cluster_id = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
  for (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster : cloud_clusters) {
    point_processor.NumPoints(cluster);
    render.RenderPointCloud(viewer, cluster, "Obstacle" + std::to_string(cluster_id), colors[cluster_id % colors.size()]);

    Box box = point_processor.BoundingBox(cluster);
    render.RenderBox(viewer, box, cluster_id);
    cluster_id++;
  }
}

void Environment::CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer) {
  ProcessPointClouds<pcl::PointXYZI> point_processor;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud = point_processor.LoadPcd("resources/data/pcd/data_1/0000000000.pcd");

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud =
      point_processor.FilterCloud(input_cloud, 0.15, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud =
      point_processor.CustomRansacSegmentPlane(filtered_cloud, 25, 0.3);

  Render render;
  render.RenderPointCloud(viewer, segment_cloud.second, "Plane Cloud", Color(0, 1, 0));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters = point_processor.CustomClustering(segment_cloud.first, .44, 3, 1000);

  int cluster_id = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(1, 1, 0)};
  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : cloud_clusters) {
    render.RenderPointCloud(viewer, cluster, "Obstacle" + std::to_string(cluster_id), colors[cluster_id % colors.size()]);

    Box box = point_processor.BoundingBox(cluster);
    render.RenderBox(viewer, box, cluster_id);
    cluster_id++;
  }
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

void Environment::CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>& point_processor,
                            const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud =
      point_processor.FilterCloud(input_cloud, 0.15, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_cloud =
      point_processor.CustomRansacSegmentPlane(filtered_cloud, 25, 0.3);

  Render render;
  render.RenderPointCloud(viewer, segment_cloud.second, "Plane Cloud", Color(0, 1, 0));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_clusters = point_processor.CustomClustering(segment_cloud.first, .44, 3, 1000);

  int cluster_id = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(1, 1, 0)};
  for (const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster : cloud_clusters) {
    render.RenderPointCloud(viewer, cluster, "Obstacle" + std::to_string(cluster_id), colors[cluster_id % colors.size()]);

    Box box = point_processor.BoundingBox(cluster);
    render.RenderBox(viewer, box, cluster_id);
    cluster_id++;
  }
}
