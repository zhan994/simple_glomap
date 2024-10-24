#include "glomap/processors/view_graph_manipulation.h"

#include "glomap/math/two_view_geometry.h"
#include "glomap/math/union_find.h"

#include <colmap/util/threading.h>

namespace glomap {

image_pair_t
ViewGraphManipulater::SparsifyGraph(ViewGraph &view_graph,
                                    std::unordered_map<image_t, Image> &images,
                                    int expected_degree) {
  image_t num_img = view_graph.KeepLargestConnectedComponents(images);

  // Keep track of chosen edges
  std::unordered_set<image_pair_t> chosen_edges;
  const std::unordered_map<image_t, std::unordered_set<image_t>>
      &adjacency_list = view_graph.GetAdjacencyList();

  // Here, the average is the mean of the degrees
  double average_degree = 0;
  for (const auto &[image_id, neighbors] : adjacency_list) {
    if (images[image_id].is_registered == false)
      continue;
    average_degree += neighbors.size();
  }
  average_degree = average_degree / num_img;

  // Go through the adjacency list and keep edge with probability
  // ((expected_degree * average_degree) / (degree1 * degree2))
  for (auto &[pair_id, image_pair] : view_graph.image_pairs) {
    if (!image_pair.is_valid)
      continue;

    image_t image_id1 = image_pair.image_id1;
    image_t image_id2 = image_pair.image_id2;

    if (images[image_id1].is_registered == false ||
        images[image_id2].is_registered == false)
      continue;

    int degree1 = adjacency_list.at(image_id1).size();
    int degree2 = adjacency_list.at(image_id2).size();

    if (degree1 <= expected_degree || degree2 <= expected_degree) {
      chosen_edges.insert(pair_id);
      continue;
    }

    if (rand() / double(RAND_MAX) <
        (expected_degree * average_degree) / (degree1 * degree2)) {
      chosen_edges.insert(pair_id);
    }
  }

  // Set all pairs not in the chosen edges to invalid
  for (auto &[pair_id, image_pair] : view_graph.image_pairs) {
    if (chosen_edges.find(pair_id) == chosen_edges.end()) {
      image_pair.is_valid = false;
    }
  }

  // Keep the largest connected component
  view_graph.KeepLargestConnectedComponents(images);

  return chosen_edges.size();
}

image_t ViewGraphManipulater::EstablishStrongClusters(
    ViewGraph &view_graph, std::unordered_map<image_t, Image> &images,
    StrongClusterCriteria criteria, double min_thres, int min_num_images) {
  image_t num_img_before = view_graph.KeepLargestConnectedComponents(images);

  // Construct the initial cluster by keeping the pairs with weight > min_thres
  UnionFind<image_pair_t> uf;
  // Go through the edges, and add the edge with weight > min_thres
  for (auto &[pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false)
      continue;

    bool status = false;
    status = status ||
             (criteria == INLIER_NUM && image_pair.inliers.size() > min_thres);
    status = status || (criteria == WEIGHT && image_pair.weight > min_thres);
    if (status) {
      uf.Union(image_pair_t(image_pair.image_id1),
               image_pair_t(image_pair.image_id2));
    }
  }

  // For every two connected components, we check the number of slightly weaker
  // pairs (> 0.75 min_thres) between them Two clusters are concatenated if the
  // number of such pairs is larger than a threshold (2)
  bool status = true;
  int iteration = 0;
  while (status) {
    status = false;
    iteration++;

    if (iteration > 10) {
      break;
    }

    std::unordered_map<image_pair_t, std::unordered_map<image_pair_t, int>>
        num_pairs;
    for (auto &[pair_id, image_pair] : view_graph.image_pairs) {
      if (image_pair.is_valid == false)
        continue;

      // If the number of inliers < 0.75 of the threshold, skip
      bool status = false;
      status = status || (criteria == INLIER_NUM &&
                          image_pair.inliers.size() < 0.75 * min_thres);
      status = status ||
               (criteria == WEIGHT && image_pair.weight < 0.75 * min_thres);
      if (status)
        continue;

      image_t image_id1 = image_pair.image_id1;
      image_t image_id2 = image_pair.image_id2;

      image_pair_t root1 = uf.Find(image_pair_t(image_id1));
      image_pair_t root2 = uf.Find(image_pair_t(image_id2));

      if (root1 == root2) {
        continue;
      }
      if (num_pairs.find(root1) == num_pairs.end())
        num_pairs.insert(
            std::make_pair(root1, std::unordered_map<image_pair_t, int>()));
      if (num_pairs.find(root2) == num_pairs.end())
        num_pairs.insert(
            std::make_pair(root2, std::unordered_map<image_pair_t, int>()));

      num_pairs[root1][root2]++;
      num_pairs[root2][root1]++;
    }
    // Connect the clusters progressively. If two clusters have more than 3
    // pairs, then connect them
    for (auto &[root1, counter] : num_pairs) {
      for (auto &[root2, count] : counter) {
        if (root1 <= root2)
          continue;

        if (count >= 2) {
          status = true;
          uf.Union(root1, root2);
        }
      }
    }
  }

  for (auto &[image_pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false)
      continue;

    image_t image_id1 = image_pair.image_id1;
    image_t image_id2 = image_pair.image_id2;

    if (uf.Find(image_pair_t(image_id1)) != uf.Find(image_pair_t(image_id2))) {
      image_pair.is_valid = false;
    }
  }
  int num_comp = view_graph.MarkConnectedComponents(images);

  LOG(INFO) << "Clustering take " << iteration << " iterations. "
            << "Images are grouped into " << num_comp
            << " clusters after strong-clustering";

  return num_comp;
}

void ViewGraphManipulater::UpdateImagePairsConfig(
    ViewGraph &view_graph, const std::unordered_map<camera_t, Camera> &cameras,
    const std::unordered_map<image_t, Image> &images) {
  // For each camera, check the number of times that the camera
  // is involved in a pair with configuration 2 'CALIBRATED'
  // First: the total occurence;
  // second: the number of pairs with configuration 2 'CALIBRATED'
  // step: 1 统计对极几何的图像对配置为E-CALIBRATED还是F-UNCALIBRATED
  std::unordered_map<camera_t, std::pair<int, int>> camera_counter;
  for (auto &[pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false)
      continue;

    camera_t camera_id1 = images.at(image_pair.image_id1).camera_id;
    camera_t camera_id2 = images.at(image_pair.image_id2).camera_id;

    const Camera &camera1 = cameras.at(camera_id1);
    const Camera &camera2 = cameras.at(camera_id2);
    if (!camera1.has_prior_focal_length || !camera2.has_prior_focal_length)
      continue;

    if (image_pair.config == colmap::TwoViewGeometry::CALIBRATED) {
      camera_counter[camera_id1].first++;
      camera_counter[camera_id2].first++;
      camera_counter[camera_id1].second++;
      camera_counter[camera_id2].second++;
    } else if (image_pair.config == colmap::TwoViewGeometry::UNCALIBRATED) {
      camera_counter[camera_id1].first++;
      camera_counter[camera_id2].first++;
    }
  }

  // Check the ratio of valid and invalid relative pair, if the majority of the
  // pairs are valid, then set the camera to valid
  // step: 2 检查图像对的有效性，E-CALIBRATED占比大即有效
  std::unordered_map<camera_t, bool> camera_validity;
  for (auto &[camera_id, counter] : camera_counter) {
    if (counter.first == 0) {
      camera_validity[camera_id] = false;
    } else if (counter.second * 1. / counter.first > 0.5) {
      camera_validity[camera_id] = true;
    } else {
      camera_validity[camera_id] = false;
    }
  }

  // step: 3 对于F-UNCALIBRATED的图像对通过帧间姿态恢复F/E
  for (auto &[pair_id, image_pair] : view_graph.image_pairs) {
    if (image_pair.is_valid == false)
      continue;
    if (image_pair.config != colmap::TwoViewGeometry::UNCALIBRATED)
      continue;

    camera_t camera_id1 = images.at(image_pair.image_id1).camera_id;
    camera_t camera_id2 = images.at(image_pair.image_id2).camera_id;

    const Camera &camera1 =
        cameras.at(images.at(image_pair.image_id1).camera_id);
    const Camera &camera2 =
        cameras.at(images.at(image_pair.image_id2).camera_id);

    if (camera_validity[camera_id1] && camera_validity[camera_id2]) {
      image_pair.config = colmap::TwoViewGeometry::CALIBRATED;

      // note: F矩阵用单位矩阵变换恢复重置？ 内参可信时强制使用E矩阵
      std::cout
          << "FundamentalFromMotionAndCameras by image_pair.cam2_from_cam1:"
          << image_pair.cam2_from_cam1.ToMatrix().matrix() << std::endl;

      FundamentalFromMotionAndCameras(camera1, camera2,
                                      image_pair.cam2_from_cam1, &image_pair.F);
    }
  }
}

// Decompose the relative camera postion from the camera config
void ViewGraphManipulater::DecomposeRelPose(
    ViewGraph &view_graph, std::unordered_map<camera_t, Camera> &cameras,
    std::unordered_map<image_t, Image> &images) {
  // step: 1 采集有效图像对
  std::vector<image_pair_t> image_pair_ids;
  for (auto &[pair_id, image_pair] : view_graph.image_pairs) {
    // step: 1.1 图像对是否有效，UNDEFINED/DEGENERATE/WATERMARK/MULTIPLE无效
    if (image_pair.is_valid == false)
      continue;

    // step: 1.2 检查是否都有相机的先验focal_length
    if (!cameras[images[image_pair.image_id1].camera_id]
             .has_prior_focal_length ||
        !cameras[images[image_pair.image_id2].camera_id].has_prior_focal_length)
      continue;
    image_pair_ids.push_back(pair_id);
  }

  const int64_t num_image_pairs = image_pair_ids.size();
  LOG(INFO) << "Decompose relative pose for " << num_image_pairs << " pairs";

  // step: 2 开启线程池处理 two_view_geometry
  colmap::ThreadPool thread_pool(colmap::ThreadPool::kMaxNumThreads);
  for (int64_t idx = 0; idx < num_image_pairs; idx++) {
    thread_pool.AddTask([&, idx]() {
      // step: 2.1 读取图像对信息
      ImagePair &image_pair = view_graph.image_pairs.at(image_pair_ids[idx]);
      image_t image_id1 = image_pair.image_id1;
      image_t image_id2 = image_pair.image_id2;

      camera_t camera_id1 = images.at(image_id1).camera_id;
      camera_t camera_id2 = images.at(image_id2).camera_id;

      // Use the two-view geometry to re-estimate the relative pose
      // step: 2.2 利用 two_view_geometry 重新估计 rel pose
      colmap::TwoViewGeometry two_view_geometry;
      two_view_geometry.E = image_pair.E;
      two_view_geometry.F = image_pair.F;
      two_view_geometry.H = image_pair.H;
      two_view_geometry.config = image_pair.config;
      // note: 使用E/H估计
      colmap::EstimateTwoViewGeometryPose(
          cameras[camera_id1], images[image_id1].features, cameras[camera_id2],
          images[image_id2].features, &two_view_geometry);

      // if it planar, then use the estimated relative pose
      if (image_pair.config == colmap::TwoViewGeometry::PLANAR &&
          cameras[camera_id1].has_prior_focal_length &&
          cameras[camera_id2].has_prior_focal_length) {
        image_pair.config = colmap::TwoViewGeometry::CALIBRATED;
        return;
      } else if (!(cameras[camera_id1].has_prior_focal_length &&
                   cameras[camera_id2].has_prior_focal_length))
        return;

      // step: 2.3 使用重新估计的外参以及two_view_geometry配置
      image_pair.config = two_view_geometry.config;
      image_pair.cam2_from_cam1 = two_view_geometry.cam2_from_cam1;

      // step: 2.4 尺度归一化
      if (image_pair.cam2_from_cam1.translation.norm() > EPS) {
        image_pair.cam2_from_cam1.translation =
            image_pair.cam2_from_cam1.translation.normalized();
      }
    });
  }

  thread_pool.Wait();

  std::size_t counter = 0;
  for (std::size_t idx = 0; idx < image_pair_ids.size(); idx++) {
    ImagePair &image_pair = view_graph.image_pairs.at(image_pair_ids[idx]);

    std::cout << "DecomposeRelPose: "
              << image_pair.cam2_from_cam1.ToMatrix().matrix() << std::endl;

    if (image_pair.config != colmap::TwoViewGeometry::CALIBRATED &&
        image_pair.config != colmap::TwoViewGeometry::PLANAR_OR_PANORAMIC)
      counter++;
  }
  LOG(INFO) << "Decompose relative pose done. " << counter
            << " pairs are pure rotation";
}

} // namespace glomap
