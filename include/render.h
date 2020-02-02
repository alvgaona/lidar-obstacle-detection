#ifndef RENDER_H_
#define RENDER_H_

#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <string>
#include <vector>

#include "box.h"

struct Color {
  float r, g, b;

  Color(float setR, float setG, float setB) : r(setR), g(setG), b(setB) {}
};

struct Vect3 {
  double x, y, z;

  Vect3(double setX, double setY, double setZ) : x(setX), y(setY), z(setZ) {}

  Vect3 operator+(const Vect3 &vec) {
    Vect3 result(x + vec.x, y + vec.y, z + vec.z);
    return result;
  }
};

enum CameraAngle { XY, TopDown, Side, FPS };

struct Car {
  // units in meters
  Vect3 position, dimensions;

  std::string name;
  Color color;

  Car(Vect3 set_position, Vect3 set_dimensions, Color set_color, std::string set_name)
      : position(set_position), dimensions(set_dimensions), color(set_color), name(set_name) {}

  void Render(pcl::visualization::PCLVisualizer::Ptr &viewer) {
    // render bottom of car
    viewer->addCube(position.x - dimensions.x / 2, position.x + dimensions.x / 2, position.y - dimensions.y / 2,
                    position.y + dimensions.y / 2, position.z, position.z + dimensions.z * 2 / 3, color.r, color.g, color.b, name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name);
    // render top of car
    viewer->addCube(position.x - dimensions.x / 4, position.x + dimensions.x / 4, position.y - dimensions.y / 2,
                    position.y + dimensions.y / 2, position.z + dimensions.z * 2 / 3, position.z + dimensions.z, color.r, color.g, color.b,
                    name + "Top");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                        pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, name + "Top");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name + "Top");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, name + "Top");
  }

  // collision helper function
  bool Inbetween(double point, double center, double range) { return (center - range <= point) && (center + range >= point); }

  bool CheckCollision(Vect3 point) {
    return (Inbetween(point.x, position.x, dimensions.x / 2) && Inbetween(point.y, position.y, dimensions.y / 2) &&
            Inbetween(point.z, position.z + dimensions.z / 3, dimensions.z / 3)) ||
           (Inbetween(point.x, position.x, dimensions.x / 4) && Inbetween(point.y, position.y, dimensions.y / 2) &&
            Inbetween(point.z, position.z + dimensions.z * 5 / 6, dimensions.z / 6));
  }
};

class Render {
 public:
  Render() = default;
  ~Render() = default;

  void RenderHighway(pcl::visualization::PCLVisualizer::Ptr &viewer);
  void RenderRays(pcl::visualization::PCLVisualizer::Ptr &viewer, const Vect3 &origin, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
  void ClearRays(pcl::visualization::PCLVisualizer::Ptr &viewer);
  void RenderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string name,
                        Color color = Color(1, 1, 1));
  void RenderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer, const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, std::string name,
                        Color color = Color(-1, -1, -1));
  void RenderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, Box box, int id, Color color = Color(1, 0, 0), float opacity = 1);
  void RenderBox(pcl::visualization::PCLVisualizer::Ptr &viewer, BoxQ box, int id, Color color = Color(1, 0, 0), float opacity = 1);
};

#endif
