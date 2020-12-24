/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include <fstream>
#include <memory>
#include "json.hpp"
#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

struct Params {
  float filterRes;
  Eigen::Vector4f minPoint;
  Eigen::Vector4f maxPoint;
  float clusterTol;
  int clusterMinSize;
  int clusterMaxSize;

  Params() = default;
  ~Params() = default;
};

Params params;

void readConfig() {
  using json = nlohmann::json;

  std::ifstream json_stream("../src/config/params.json");
  json params_json;
  json_stream >> params_json;
  json_stream.close();

  params.filterRes = params_json["filterRes"];
  params.minPoint =
      Eigen::Vector4f(params_json["minPoint"][0], params_json["minPoint"][1],
                      params_json["minPoint"][2], 1);
  params.maxPoint =
      Eigen::Vector4f(params_json["maxPoint"][0], params_json["maxPoint"][1],
                      params_json["maxPoint"][2], 1);
  params.clusterTol = params_json["clusterTol"];
  params.clusterMinSize = params_json["clusterMinSize"];
  params.clusterMaxSize = params_json["clusterMaxSize"];
}

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr& viewer) {
  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = false;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // TODO:: Create lidar sensor
  std::shared_ptr<Lidar> lidar(new Lidar(cars, 0));
  auto point_cloud = lidar->scan();
  // renderRays(viewer, lidar->position, point_cloud);
  // renderPointCloud(viewer, point_cloud, "point_cloud");

  // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ> point_processor;
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
            pcl::PointCloud<pcl::PointXYZ>::Ptr>
      segmentCloud = point_processor.SegmentPlane(point_cloud, 100, 0.2);
  // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(1, 1, 1));

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
      point_processor.ClusteringPcl(segmentCloud.first, 2.0, 3, 40);
  int clusterId = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size ";
    point_processor.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                     colors[clusterId]);
    ++clusterId;
  }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>& point_processor,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

  // Experiment with the ? values and find what works best
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud =
      point_processor.FilterCloud(inputCloud, params.filterRes, params.minPoint,
                                  params.maxPoint);
  // renderPointCloud(viewer, filterCloud, "filterCloud");

  // Segmentation
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr>
      segmentCloudPcl = point_processor.SegmentPlanePcl(filterCloud, 100, 0.2);

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,
            pcl::PointCloud<pcl::PointXYZI>::Ptr>
      segmentCloud = point_processor.SegmentPlane(filterCloud, 100, 0.2);
  // renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(0, 1, 0));
  renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(1, 1, 1));

  // Cluster
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClustersPcl =
      point_processor.ClusteringPcl(segmentCloud.first, params.clusterTol,
                                    params.clusterMinSize,
                                    params.clusterMaxSize);

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
      point_processor.Clustering(segmentCloud.first, params.clusterTol,
                                 params.clusterMinSize, params.clusterMaxSize);

  // Render each detected obstacle's point cloud and bounding box.
  int clusterId = 0;
  std::vector<Color> colors = {
      Color(0.0078, 0.651, 0.353), Color(0.906, 0.902, 0),
      Color(0.902, 0.427, 0.0078), Color(0.843, 0.4274, 0.859),
      Color(0, 0.2157, 0.68),      Color(0.808, 1, 0.694),
      Color(0.835, 0.93, 0.302),   Color(1, 0.463, 0.33),
      Color(0.992, 0.706, 0.318),  Color(0.627, 0, 0.396)};
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    // std::cout << "cluster size ";
    // point_processor.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstCloud" + std::to_string(clusterId),
                     colors[clusterId % colors.size()]);

    Box box = point_processor.BoundingBox(cluster);
    renderBox(viewer, box, clusterId, Color(1, 0, 0), 1);
    ++clusterId;
  }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr& viewer) {
  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
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

  if (setAngle != FPS) {
    viewer->addCoordinateSystem(1.0);
  }
}

int main(int argc, char** argv) {
  std::cout << "starting enviroment" << std::endl;
  readConfig();

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  ProcessPointClouds<pcl::PointXYZI> point_processor;
  std::vector<boost::filesystem::path> stream =
      point_processor.streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  while (!viewer->wasStopped()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    inputCloudI = point_processor.loadPcd((*streamIterator).string());
    cityBlock(viewer, point_processor, inputCloudI);

    streamIterator++;
    if (streamIterator == stream.end()) {
      streamIterator = stream.begin();
      std::cout << "Reset stream." << std::endl;
      std::cout << std::endl;
    }

    viewer->spinOnce();
  }
}