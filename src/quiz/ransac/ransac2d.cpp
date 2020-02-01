#include <cstdlib>
#include <ctime>
#include <unordered_set>

#include "../../../include/process_point_clouds.h"
#include "../../../include/render.h"
#include "../../process_point_clouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int num_outliers = 10;
  while (num_outliers--) {
    double rx = 2 * (((double)std::rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)std::rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.LoadPcd("../../../../resources/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr InitScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int max_iterations, float distance_tolerance) {
  auto start_time = std::chrono::steady_clock::now();

  std::unordered_set<int> inliers_result;
  std::srand(std::time(0));

  while (max_iterations--) {
    std::unordered_set<int> inliers;

    while (inliers.size() < 2) {
      inliers.insert(std::rand() % cloud->points.size());
    }

    auto itr = inliers.begin();
    float x1 = cloud->points[*itr].x;
    float y1 = cloud->points[*itr].y;
    itr++;
    float x2 = cloud->points[*itr].x;
    float y2 = cloud->points[*itr].y;

    float a = (y1 - y2);
    float b = (x2 - x1);
    float c = (x1 * y2 - x2 * y1);

    for (auto i = 0; i < cloud->points.size(); i++) {
      if (inliers.count(i) > 0) {
        continue;
      }

      pcl::PointXYZ point = cloud->points[i];
      float x3 = point.x;
      float y3 = point.y;

      float d = fabs(a * x3 + b * y3 + c) / sqrt(a * a + b * b);

      if (d <= distance_tolerance) {
        inliers.insert(i);
      }
    }

    if (inliers.size() > inliers_result.size()) {
      inliers_result = inliers;
    }
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "RANSAC took " << elapsed_time.count() << " ms." << std::endl;

  return inliers_result;
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int max_iterations, float distance_tolerance) {
  auto start_time = std::chrono::steady_clock::now();

  std::unordered_set<int> inliers_result;
  std::srand(std::time(0));

  while (max_iterations--) {
    std::unordered_set<int> inliers;

    while (inliers.size() < 3) {
      inliers.insert(std::rand() % cloud->points.size());
    }

    auto itr = inliers.begin();
    float x1 = cloud->points[*itr].x;
    float y1 = cloud->points[*itr].y;
    float z1 = cloud->points[*itr].z;
    itr++;
    float x2 = cloud->points[*itr].x;
    float y2 = cloud->points[*itr].y;
    float z2 = cloud->points[*itr].z;
    itr++;
    float x3 = cloud->points[*itr].x;
    float y3 = cloud->points[*itr].y;
    float z3 = cloud->points[*itr].z;

    float a = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
    float b = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
    float c = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
    float d = -(a * x1 + b * y1 + c * z1);

    for (auto i = 0; i < cloud->points.size(); i++) {
      if (inliers.count(i) > 0) {
        continue;
      }

      pcl::PointXYZ point = cloud->points[i];
      float x4 = point.x;
      float y4 = point.y;
      float z4 = point.z;

      float d = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

      if (d <= distance_tolerance) {
        inliers.insert(i);
      }
    }

    if (inliers.size() > inliers_result.size()) {
      inliers_result = inliers;
    }
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "RANSAC 3D took " << elapsed_time.count() << " ms." << std::endl;

  return inliers_result;
}

int main() {
  pcl::visualization::PCLVisualizer::Ptr viewer = InitScene();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
  std::unordered_set<int> inliers = RansacPlane(cloud, 10, 0.5);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloud_inliers->points.push_back(point);
    else
      cloud_outliers->points.push_back(point);
  }

  Render render;
  std::cout << inliers.size() << std::endl;

  if (inliers.size()) {
    render.RenderPointCloud(viewer, cloud_inliers, "inliers", Color(0, 1, 0));
    render.RenderPointCloud(viewer, cloud_outliers, "outliers", Color(1, 0, 0));
  } else {
    render.RenderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
