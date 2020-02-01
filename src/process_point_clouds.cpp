#include "../include/process_point_clouds.h"

#include "cluster.cpp"
#include "kd_tree.cpp"

template <typename PointT>
void ProcessPointClouds<PointT>::NumPoints(typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float resolution,
                                                                              Eigen::Vector4f min_point, Eigen::Vector4f max_point) {
  // Time segmentation process
  auto start_time = std::chrono::steady_clock::now();

  pcl::VoxelGrid<PointT> voxel_grid;
  typename pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(resolution, resolution, resolution);
  voxel_grid.filter(*filtered_cloud);

  typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);

  pcl::CropBox<PointT> region(true);
  region.setMin(min_point);
  region.setMax(max_point);
  region.setInputCloud(filtered_cloud);
  region.filter(*cloud_region);

  std::vector<int> indices;

  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roof.setInputCloud(cloud_region);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

  for (auto& point : indices) {
    inliers->indices.emplace_back(point);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_region);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_region);

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "Filtering took " << elapsed_time.count() << " ms" << std::endl;
  std::cout << "Number of points after filtering: " << cloud_region->points.size() << std::endl;

  return cloud_region;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) {
  typename pcl::PointCloud<PointT>::Ptr obstacle_cloud(new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr plane_cloud(new pcl::PointCloud<PointT>());

  for (auto& inlier : inliers->indices) {
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

  if (!inliers->indices.empty()) {
    std::cout << "Planar model could not be estimated based on the given dataset" << std::endl;
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "Plane segmentation took " << elapsed_time.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segment_result = SeparateClouds(inliers, cloud);
  return segment_result;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::CustomRansacSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations,
                                                     float distance_threshold) {
  // Time segmentation process
  auto start_time = std::chrono::steady_clock::now();
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};

  std::unordered_set<int> inlier_indices = RansacPlane(cloud, max_iterations, distance_threshold);

  for (auto& index : inlier_indices) {
    inliers->indices.emplace_back(index);
  }

  if (!inliers->indices.empty()) {
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

  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> euclidean_cluster;
  euclidean_cluster.setClusterTolerance(cluster_tolerance);
  euclidean_cluster.setMinClusterSize(min_size);
  euclidean_cluster.setMaxClusterSize(max_size);
  euclidean_cluster.setSearchMethod(tree);
  euclidean_cluster.setInputCloud(cloud);
  euclidean_cluster.extract(cluster_indices);

  for (auto& cluster_index : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

    for (auto& index : cluster_index.indices) {
      cloud_cluster->points.emplace_back(cloud->points[index]);
    }

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.emplace_back(cloud_cluster);
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "Clustering took " << elapsed_time.count() << " ms and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::CustomClustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                                                                                                float cluster_tolerance, int min_size,
                                                                                                int max_size) {
  auto start_time = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  std::unique_ptr<KdTree<PointT>> tree = std::make_unique<KdTree<PointT>>();
  tree->SetInputCloud(cloud);

  std::vector<std::vector<int>> ec_result = EuclideanCluster(cloud, tree.get(), cluster_tolerance, min_size, max_size);

  std::vector<pcl::PointIndices> cluster_indices;
  for (auto& c : ec_result) {
    pcl::PointIndices indices;
    indices.indices = c;
    cluster_indices.emplace_back(indices);
  }

  for (auto& cluster_index : cluster_indices) {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);

    for (auto& index : cluster_index.indices) {
      cloud_cluster->points.emplace_back(cloud->points[index]);
    }

    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.emplace_back(cloud_cluster);
  }

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  std::cout << "Clustering took " << elapsed_time.count() << " ms and found " << clusters.size() << " clusters" << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster) {
  // Find bounding box for one of the clusters
  PointT minPoint, maxPoint;
  pcl::getMinMax3D(*cluster, minPoint, maxPoint);

  Box box{};
  box.x_min = minPoint.x;
  box.y_min = minPoint.y;
  box.z_min = minPoint.z;
  box.x_max = maxPoint.x;
  box.y_max = maxPoint.y;
  box.z_max = maxPoint.z;

  return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::SavePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
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

template <typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int max_iterations,
                                                                float distance_tolerance) {
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

      PointT point = cloud->points[i];
      float x4 = point.x;
      float y4 = point.y;
      float z4 = point.z;

      float distance = fabs(a * x4 + b * y4 + c * z4 + d) / sqrt(a * a + b * b + c * c);

      if (distance <= distance_tolerance) {
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
