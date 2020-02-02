#ifndef LIDAR_H_
#define LIDAR_H_

#include <chrono>
#include <ctime>

#include "render.h"

const double pi = 3.1415;

struct Ray {
  Vect3 origin;
  double resolution;
  Vect3 direction;
  Vect3 castPosition;
  double castDistance;

  // parameters:
  // setOrigin: the starting position from where the ray is cast
  // horizontalAngle: the angle of direction the ray travels on the xy plane
  // verticalAngle: the angle of direction between xy plane and ray
  // 				  for example 0 radians is along xy plane and pi/2 radians
  // is stright up resoultion: the magnitude of the ray's step, used for ray
  // casting, the smaller the more accurate but the more expensive

  Ray(Vect3 setOrigin, double horizontalAngle, double verticalAngle, double setResolution)
      : origin(setOrigin),
        resolution(setResolution),
        direction(resolution * cos(verticalAngle) * cos(horizontalAngle), resolution * cos(verticalAngle) * sin(horizontalAngle),
                  resolution * sin(verticalAngle)),
        castPosition(origin),
        castDistance(0) {}

  void RayCast(const std::vector<Car> &cars, double minDistance, double maxDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
               double slopeAngle, double sderr) {
    // reset ray
    castPosition = origin;
    castDistance = 0;

    bool collision = false;

    while (!collision && castDistance < maxDistance) {
      castPosition = castPosition + direction;
      castDistance += resolution;

      // check if there is any collisions with ground slope
      collision = (castPosition.z <= castPosition.x * tan(slopeAngle));

      // check if there is any collisions with cars
      if (!collision && castDistance < maxDistance) {
        for (Car car : cars) {
          collision |= car.CheckCollision(castPosition);
          if (collision) break;
        }
      }
    }

    if ((castDistance >= minDistance) && (castDistance <= maxDistance)) {
      // add noise based on standard deviation error
      double rx = ((double)rand() / (RAND_MAX));
      double ry = ((double)rand() / (RAND_MAX));
      double rz = ((double)rand() / (RAND_MAX));
      cloud->points.emplace_back(castPosition.x + rx * sderr, castPosition.y + ry * sderr, castPosition.z + rz * sderr);
    }
  }
};

struct Lidar {
  std::vector<Ray> rays;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<Car> cars;
  Vect3 position;
  double ground_slope;
  double min_distance;
  double max_distance;
  double resoultion;
  double sderr;

  Lidar(std::vector<Car> set_cars, double set_ground_slope) : cloud(new pcl::PointCloud<pcl::PointXYZ>()), position(0, 0, 2.6) {
    min_distance = 5;
    max_distance = 50;
    resoultion = 0.2;
    sderr = 0.2;
    cars = set_cars;
    ground_slope = set_ground_slope;

    int num_layers = 8;
    // the steepest vertical angle
    double steepest_angle = 30.0 * (-pi / 180);
    double angle_range = 26.0 * (pi / 180);
    double horizontal_angle_inc = pi / 64;

    double angleIncrement = angle_range / num_layers;

    for (double angle_vertical = steepest_angle; angle_vertical < steepest_angle + angle_range; angle_vertical += angleIncrement) {
      for (double angle = 0; angle <= 2 * pi; angle += horizontal_angle_inc) {
        Ray ray(position, angle, angle_vertical, resoultion);
        rays.push_back(ray);
      }
    }
  }

  ~Lidar() {}

  pcl::PointCloud<pcl::PointXYZ>::Ptr Scan() {
    cloud->points.clear();
    auto start_time = std::chrono::steady_clock::now();
    for (Ray ray : rays) ray.RayCast(cars, min_distance, max_distance, cloud, ground_slope, sderr);
    auto end_time = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    cout << "Ray casting took " << elapsed_time.count() << " milliseconds" << endl;
    cloud->width = cloud->points.size();
    cloud->height = 1;  // one dimensional unorganized point cloud dataset
    return cloud;
  }
};

#endif
