// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region
  // based filtering
  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(
      new pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(filterRes, filterRes, filterRes);
  sor.filter(*cloud_filtered);

  typename pcl::PointCloud<PointT>::Ptr cloudRegion(
      new pcl::PointCloud<PointT>());
  pcl::CropBox<PointT> crop(true);
  crop.setMin(minPoint);
  crop.setMax(maxPoint);
  crop.setInputCloud(cloud_filtered);
  // crop.setKeepOrganized(true);
  // crop.setUserFilterValue(0.1f);
  crop.filter(*cloudRegion);

  std::vector<int> roof_indices;
  pcl::CropBox<PointT> crop_roof(true);
  crop_roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  crop_roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  crop_roof.setInputCloud(cloudRegion);
  crop_roof.filter(roof_indices);

  pcl::PointIndices::Ptr roof_inliers(new pcl::PointIndices());
  for (auto idx : roof_indices) {
    roof_inliers->indices.push_back(idx);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloudRegion);
  extract.setIndices(roof_inliers);
  extract.setNegative(true);
  extract.filter(*cloudRegion);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloudRegion;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane
  typename pcl::PointCloud<PointT>::Ptr cloud_obs(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr cloud_plane(
      new pcl::PointCloud<PointT>());

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_plane);
  // Another way
  /* for (auto idx : inliers->indices) {
    cloud_plane->points.push_back(cloud->points[idx]);
  } */

  extract.setNegative(true);
  extract.filter(*cloud_obs);

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult(cloud_obs, cloud_plane);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlanePcl(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  // TODO:: Fill in this function to find inliers for the cloud.
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  // pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<PointT> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIterations);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "PCL plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(inliers, cloud);
  return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<int> inliersResult;
  // Using RANSAC algorithm to segment plane
  // For max iterations
  std::random_device rd;
  std::default_random_engine gen = std::default_random_engine(rd());
  std::uniform_int_distribution<int> dis(0, cloud->points.size());

  double plane_time = 0.0;
  double ransac_time = 0.0;
  for (int i = 0; i < maxIterations; ++i) {
    std::vector<int> inliers;
    std::vector<int> rand_idx;
    // Randomly sample subset and fit plane
    while (rand_idx.size() < 3) {
      int idx = dis(gen);
      if (rand_idx.size() == 0) {
        rand_idx.push_back(idx);
      } else if (rand_idx.size() == 1 && idx != rand_idx[0]) {
        rand_idx.push_back(idx);
      } else if (rand_idx.size() == 2 && idx != rand_idx[0] &&
                 idx != rand_idx[1]) {
        rand_idx.push_back(idx);
      }
    }

    PointT& p1 = cloud->points[rand_idx[0]];
    PointT& p2 = cloud->points[rand_idx[1]];
    PointT& p3 = cloud->points[rand_idx[2]];
    std::array<float, 3> vec1 = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
    std::array<float, 3> vec2 = {p3.x - p1.x, p3.y - p1.y, p3.z - p1.z};
    std::array<float, 3> v1_cross_v2 = {vec1[1] * vec2[2] - vec1[2] * vec2[1],
                                        vec1[2] * vec2[0] - vec1[0] * vec2[2],
                                        vec1[0] * vec2[1] - vec1[1] * vec2[0]};
    // Plane equation: Ax + By + Cz + D = 0
    float A = v1_cross_v2[0];
    float B = v1_cross_v2[1];
    float C = v1_cross_v2[2];
    float D = -(A * p1.x + B * p1.y + C * p1.z);

    // Measure distance between every point and fitted line
    float temp = std::sqrt(A * A + B * B + C * C);
    // Performance improve:
    // std::fabs(A * p.x + B * p.y + C * p.z + D) / temp < distanceThreshold
    // is equal to
    // std::fabs(A * p.x + B * p.y + C * p.z + D) < distanceThreshold * temp
    float threshold = distanceThreshold * temp;
    for (size_t i = 0; i < cloud->points.size(); i++) {
      // If distance is smaller than threshold, count it as inlier
      PointT& p = cloud->points[i];
      // float dist = std::fabs(A * p.x + B * p.y + C * p.z + D) / temp;
      float dist = std::fabs(A * p.x + B * p.y + C * p.z + D);
      if (dist < threshold) {
        inliers.push_back(i);
      }
    }

    if (inliers.size() > inliersResult.size()) {
      // inliersResult = inliers;
      inliersResult.swap(inliers);
    }
  }

  pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices());
  plane_inliers->indices.swap(inliersResult);
  // for (int idx : inliersResult) {
  //   plane_inliers->indices.push_back(idx);
  // }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
      segResult = SeparateClouds(plane_inliers, cloud);

  return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<
    PointT>::ClusteringPcl(typename pcl::PointCloud<PointT>::Ptr cloud,
                           float clusterTolerance, int minSize, int maxSize) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree(
      new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(cluster_indices);
  for (std::vector<pcl::PointIndices>::const_iterator cluster_iter =
           cluster_indices.begin();
       cluster_iter != cluster_indices.end(); ++cluster_iter) {
    typename pcl::PointCloud<PointT>::Ptr cloud_cluster(
        new pcl::PointCloud<PointT>);
    for (std::vector<int>::const_iterator pit = cluster_iter->indices.begin();
         pit != cluster_iter->indices.end(); ++pit) {
      // Access point in cloud
      // cloud_cluster->push_back((*cloud)[*pit]);
      // Another way to access point in cloud
      cloud_cluster->push_back(cloud->points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    clusters.push_back(cloud_cluster);
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "PCL clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<
    PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud,
                        float clusterTolerance, int minSize, int maxSize) {
  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

  // Build KdTree
  // auto t1 = std::chrono::steady_clock::now();
  KdTree::Ptr tree(new KdTree(3));
  std::vector<int> idx_vec;
  for (size_t i = 0; i < cloud->points.size(); i++) {
    idx_vec.push_back(i);
  }
  // Shuffle the order of point cloud to let kdtree keep balence.
  std::random_device rd;
  std::shuffle(idx_vec.begin(), idx_vec.end(),
               std::default_random_engine(rd()));
  for (int i = 0; i < idx_vec.size(); i++) {
    int idx = idx_vec[i];
    std::vector<float> point = {cloud->points[idx].x, cloud->points[idx].y,
                                cloud->points[idx].z};
    tree->insert(point, idx);
  }
  // auto t2 = std::chrono::steady_clock::now();

  // Euclidean cluster
  std::vector<int> processedPoints(cloud->points.size(), 0);
  for (int i = 0; i < cloud->points.size(); ++i) {
    if (processedPoints[i] == 0) {
      std::list<int> cluster_idx;
      // auto p1 = std::chrono::steady_clock::now();
      proximity(cloud, i, tree, clusterTolerance, cluster_idx, processedPoints);
      // auto p2 = std::chrono::steady_clock::now();
      // std::chrono::duration<double> time_diff = p2 - p1;
      // std::cout << "new cluster: " << time_diff.count() * 1000 << std::endl;

      if (cluster_idx.size() > minSize && cluster_idx.size() < maxSize) {
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud(
            new pcl::PointCloud<PointT>);
        for (int id : cluster_idx) {
          cluster_cloud->push_back(cloud->points[id]);
        }

        clusters.push_back(cluster_cloud);
      }
    }
  }
  // auto t3 = std::chrono::steady_clock::now();

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  // std::chrono::duration<double> time_diff = t2 - t1;
  // std::cout << "kdtree: " << time_diff.count() * 1000 << " milliseconds"
  //           << std::endl;
  // time_diff = t3 - t2;
  // std::cout << "cluster: " << time_diff.count() * 1000 << " milliseconds"
  //           << std::endl;
  // std::cout << "kdtree depth: " << tree->depth_ << std::endl;

  return clusters;
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(
    typename pcl::PointCloud<PointT>::Ptr cloud, int id, KdTree::Ptr tree,
    float distanceTol, std::list<int>& cluster_idx,
    std::vector<int>& processedPoints) {
  processedPoints[id] = 1;
  cluster_idx.push_back(id);
  std::vector<float> target = {cloud->points[id].x, cloud->points[id].y,
                               cloud->points[id].z};
  std::vector<int> nearby = tree->search(target, distanceTol);
  for (int i = 0; i < nearby.size(); ++i) {
    if (processedPoints[nearby[i]] == 0) {
      proximity(cloud, nearby[i], tree, distanceTol, cluster_idx,
                processedPoints);
    }
  }
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {
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
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(
    std::string file) {
  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)  //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(
    std::string dataPath) {
  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}