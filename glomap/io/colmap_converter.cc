#include "glomap/io/colmap_converter.h"
#include "glomap/math/two_view_geometry.h"

namespace glomap {

// For ease of debug, go through the database twice: first extract the available
// pairs, then read matches from pairs.
void ConvertDatabaseToGlomap(const colmap::Database &database,
                             ViewGraph &view_graph,
                             std::unordered_map<camera_t, Camera> &cameras,
                             std::unordered_map<image_t, Image> &images) {
  // step: 1 Add the images
  std::vector<colmap::Image> images_colmap = database.ReadAllImages();
  image_t counter = 0;
  for (auto &image : images_colmap) {
    std::cout << "\r Loading Images " << counter + 1 << " / "
              << images_colmap.size() << std::flush;
    counter++;

    image_t image_id = image.ImageId();
    if (image_id == colmap::kInvalidImageId)
      continue;
    auto ite = images.insert(std::make_pair(
        image_id, Image(image_id, image.CameraId(), image.Name())));

    // todo: add pose_prior by txt/csv file
    const colmap::PosePrior prior = database.ReadPosePrior(image_id);
    std::cout << "PosePrior info: " << prior.position << std::endl;
    if (prior.IsValid()) {
      ite.first->second.cam_from_world = Rigid3d(
          colmap::Rigid3d(Eigen::Quaterniond::Identity(), prior.position));
    } else {
      ite.first->second.cam_from_world = Rigid3d();
    }
  }
  std::cout << std::endl;

  // step: 2 Read keypoints
  for (auto &[image_id, image] : images) {
    colmap::FeatureKeypoints keypoints = database.ReadKeypoints(image_id);

    image.features.reserve(keypoints.size());
    for (int i = 0; i < keypoints.size(); i++) {
      image.features.emplace_back(
          Eigen::Vector2d(keypoints[i].x, keypoints[i].y));
    }
  }

  // step: 3 Add the cameras
  std::vector<colmap::Camera> cameras_colmap = database.ReadAllCameras();
  for (auto &camera : cameras_colmap) {
    camera_t camera_id = camera.camera_id;
    cameras[camera_id] = camera;
  }

  // step: 4 Add the matches & view_graph via matches
  std::vector<std::pair<colmap::image_pair_t, colmap::FeatureMatches>>
      all_matches = database.ReadAllMatches();

  // Go through all matches and store the matche with enough observations in the
  // view_graph
  std::size_t invalid_count = 0;
  std::unordered_map<image_pair_t, ImagePair> &image_pairs =
      view_graph.image_pairs;
  for (std::size_t match_idx = 0; match_idx < all_matches.size(); match_idx++) {
    if ((match_idx + 1) % 1000 == 0 || match_idx == all_matches.size() - 1)
      std::cout << "\r Loading Image Pair " << match_idx + 1 << " / "
                << all_matches.size() << std::flush;

    // step: 4.1 Read the image pair
    colmap::image_pair_t pair_id = all_matches[match_idx].first;
    std::pair<colmap::image_t, colmap::image_t> image_pair_colmap =
        database.PairIdToImagePair(pair_id);
    colmap::image_t image_id1 = image_pair_colmap.first;
    colmap::image_t image_id2 = image_pair_colmap.second;

    // step: 4.2 Read feature matches
    colmap::FeatureMatches &feature_matches = all_matches[match_idx].second;

    // Initialize the image pair
    // step: 4.3 two_view_geometry for image_pair
    auto ite = image_pairs.insert(
        std::make_pair(ImagePair::ImagePairToPairId(image_id1, image_id2),
                       ImagePair(image_id1, image_id2)));
    ImagePair &image_pair = ite.first->second;

    colmap::TwoViewGeometry two_view =
        database.ReadTwoViewGeometry(image_id1, image_id2);

    // If the image is marked as invalid or watermark, then skip
    if (two_view.config == colmap::TwoViewGeometry::UNDEFINED ||
        two_view.config == colmap::TwoViewGeometry::DEGENERATE ||
        two_view.config == colmap::TwoViewGeometry::WATERMARK ||
        two_view.config == colmap::TwoViewGeometry::MULTIPLE) {
      image_pair.is_valid = false;
      invalid_count++;
      continue;
    }

    // step: 4.3.1 matrixF / matrixH (planar)
    if (two_view.config == colmap::TwoViewGeometry::UNCALIBRATED) {
      image_pair.F = two_view.F;
    } else if (two_view.config == colmap::TwoViewGeometry::CALIBRATED) {
      // note: F矩阵用单位矩阵变换恢复重置？ 内参可信时强制使用E矩阵
      std::cout
          << "FundamentalFromMotionAndCameras by two_view.cam2_from_cam1: "
          << two_view.cam2_from_cam1.ToMatrix().matrix() << std::endl;

      FundamentalFromMotionAndCameras(
          cameras.at(images.at(image_pair.image_id1).camera_id),
          cameras.at(images.at(image_pair.image_id2).camera_id),
          two_view.cam2_from_cam1, &image_pair.F);
    } else if (two_view.config == colmap::TwoViewGeometry::PLANAR ||
               two_view.config == colmap::TwoViewGeometry::PANORAMIC ||
               two_view.config ==
                   colmap::TwoViewGeometry::PLANAR_OR_PANORAMIC) {
      image_pair.H = two_view.H;
      image_pair.F = two_view.F;
    }
    image_pair.config = two_view.config;

    // step: 4.4 Collect the matches
    image_pair.matches = Eigen::MatrixXi(feature_matches.size(), 2);

    std::vector<Eigen::Vector2d> &keypoints1 =
        images[image_pair.image_id1].features;
    std::vector<Eigen::Vector2d> &keypoints2 =
        images[image_pair.image_id2].features;

    feature_t count = 0;
    for (int i = 0; i < feature_matches.size(); i++) {
      colmap::point2D_t point2D_idx1 = feature_matches[i].point2D_idx1;
      colmap::point2D_t point2D_idx2 = feature_matches[i].point2D_idx2;
      if (point2D_idx1 != colmap::kInvalidPoint2DIdx &&
          point2D_idx2 != colmap::kInvalidPoint2DIdx) {
        if (keypoints1.size() <= point2D_idx1 ||
            keypoints2.size() <= point2D_idx2)
          continue;
        image_pair.matches.row(count) << point2D_idx1, point2D_idx2;
        count++;
      }
    }
    image_pair.matches.conservativeResize(count, 2);
  }
  std::cout << std::endl;

  LOG(INFO) << "Pairs read done. " << invalid_count << " / "
            << view_graph.image_pairs.size() << " are invalid";
}

} // namespace glomap
