#include "process_point_clouds.h"

template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::NumPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filter_res,
                                                                              Eigen::Vector4f min_point, Eigen::Vector4f max_point) {
  // Time segmentation process
  auto start_time = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "filtering took " << elapsed_time.count() << " milliseconds" << std::endl;

  return cloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());

  for(auto& inlier : inliers->indices) {
    plane_cloud->points.emplace_back(cloud->points[inlier]);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstacle_cloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segment_result(obstacle_cloud, plane_cloud);
  return segment_result;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations, float distance_threshold) {
  // Time segmentation process
  auto start_time = std::chrono::steady_clock::now();
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  pcl::ModelCoefficients::Ptr coefficients{new pcl::ModelCoefficients};
  pcl::SACSegmentation<PointT> segmenter;

  segmenter.setOptimizeCoefficients(true);
  segmenter.setModelType(pcl::SACMODEL_PLANE);
  segmenter.setMethodType(pcl::SAC_RANSAC);
  segmenter.setMaxIterations(max_iterations);
  segmenter.setDistanceThreshold(distance_threshold);

  segmenter.setInputCloud(cloud);
  segmenter.segment(*inliers, *coefficients);

  if(!inliers->indices.empty()) {
    std::cout << "Planar model could not be estimated based on the given dataset" << std::endl;
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "Plane segmentation took " << elapsed_time.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segment_result = SeparateClouds(inliers, cloud);
  return segment_result;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                          float cluster_tolerance, int min_size,
                                                                                          int max_size) {
  // Time clustering process
  auto start_time = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "clustering took " << elapsed_time.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box;
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::LoadPcd(std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::StreamPcd(std::string data_path) {
  std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{data_path}, boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}
