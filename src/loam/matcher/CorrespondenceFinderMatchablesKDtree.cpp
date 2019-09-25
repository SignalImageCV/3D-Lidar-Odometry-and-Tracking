#include "./CorrespondenceFinderMatchablesKDtree.hpp"

namespace Loam{

  using namespace srrg2_core;

  void CorrespondenceFinderMatchablesKDTree::reset() {
    BaseType::reset();

    _is_initialized = false;

    if (!_fixed) {
      throw std::runtime_error("CorrespondenceFinderMatchablesKDTree::reset|error, fixed not set");
    }

    // ia initialize trees
    KdTreeTypePoints::VectorTDVector points_coordinates;
    points_coordinates.reserve(_fixed->size());
    _index_map_points.reserve(_fixed->size());
    KdTreeTypeLines::VectorTDVector lines_coordinates;
    lines_coordinates.reserve(_fixed->size());
    _index_map_lines.reserve(_fixed->size());
    KdTreeTypePlanes::VectorTDVector planes_coordinates;
    planes_coordinates.reserve(_fixed->size());
    _index_map_planes.reserve(_fixed->size());

    // ia populate
    for (size_t k = 0; k < _fixed->size(); ++k) {
      const CustomMatchablef & m = _fixed->at(k);
      switch (m.type()) {
        case MatchableBase::Type::Point: {
          points_coordinates.emplace_back(m.origin());
          _index_map_points.emplace_back(k);
          break;
        }

        case MatchableBase::Type::Line: {
          KdTreeTypeLines::VectorTD coords = KdTreeTypeLines::VectorTD::Zero();
          coords.head<3>()                 = (m.direction()).cross(m.origin());
          coords.tail<3>()                 = m.direction() * _line_direction_tree_weight;
          lines_coordinates.emplace_back(coords);
          _index_map_lines.emplace_back(k);
          break;
        }

        case MatchableBase::Type::Plane: {
          KdTreeTypePlanes::VectorTD coords = KdTreeTypePlanes::VectorTD::Zero();
          coords.head<3>()                  = m.direction() * _plane_direction_tree_weight;
          coords(3)                         = m.direction().dot(m.origin());
          planes_coordinates.emplace_back(coords);
          _index_map_planes.emplace_back(k);
          break;
        }
        default:
          std::cerr << "CorrespondenceFinderMatchablesKDTree::reset|unknown matchable type [" +
                         std::to_string((int) m.type()) + "]";
      }
    }

    // ia construct the kd trees and initialize it with the points
    _kd_tree_points.reset(new KdTreeTypePoints(
      points_coordinates, param_max_leaf_range_points.value(), param_min_leaf_points.value()));

    _kd_tree_lines.reset(new KdTreeTypeLines(
      lines_coordinates, param_max_leaf_range_points.value(), param_min_leaf_points.value()));

    _kd_tree_planes.reset(new KdTreeTypePlanes(
      planes_coordinates, param_max_leaf_range_points.value(), param_min_leaf_points.value()));



    //todo remove this, only for debugging purposes
    _kd_tree_lines->printKDTree();
    _kd_tree_planes->printKDTree();
    //end

  }

  void CorrespondenceFinderMatchablesKDTree::compute() {
    _correspondences->reserve(_moving->size());

    for (size_t k = 0; k < _moving->size(); ++k) {
      CustomMatchablef& moving_m = _moving->at(k);
      // ia transform the moving according to estimate
      CustomMatchablef moving_m_transformed = moving_m.transform(_estimate);

      // ia query the right tree
      switch (moving_m.type()) {
        case MatchableBase::Type::Point: {
          Correspondence c = _findPointAssociation(k, moving_m, moving_m_transformed);
          if (c.fixed_idx > 0 && c.moving_idx > 0) {
            _correspondences->emplace_back(c);
          }
          break;
        }
        case MatchableBase::Type::Line: {
          Correspondence c = _findLineAssociation(k, moving_m, moving_m_transformed);
          if (c.fixed_idx > 0 && c.moving_idx > 0) {
            _correspondences->emplace_back(c);
          }
          break;
        }
        case MatchableBase::Type::Plane: {
          Correspondence c = _findPlaneAssociation(k, moving_m, moving_m_transformed);
          if (c.fixed_idx > 0 && c.moving_idx > 0) {
            _correspondences->emplace_back(c);
          }
          break;
        }
        default:
          std::cerr << "CorrespondenceFinderMatchablesKDTree::compute|unknown matchable type [" +
                         std::to_string((int) moving_m.type()) + "]";
      }
    }
  }

  srrg2_core::Correspondence CorrespondenceFinderMatchablesKDTree::_findPointAssociation(
    const size_t& moving_idx_,
    CustomMatchablef& moving_,
    CustomMatchablef& moving_transformed_) {
    Correspondence c;

    KdTreeTypePoints::VectorTD responce_coords = KdTreeTypePoints::VectorTD::Zero();
    int responce_idx                           = -1;


    // ia query the tree
    //    const float responce_distance =
    _kd_tree_points->findNeighbor(responce_coords,
                                  responce_idx,
                                  moving_transformed_.origin(),
                                  param_max_leaf_range_points.value());

    // ia if kdtree fails return invalid correspondence
    if (responce_idx < 0) {
      ++_stats.non_associated;
      return c;
    }

    // ia get the matchable from the fixed
    const CustomMatchablef& responce_matchable = _fixed->at(_index_map_points[responce_idx]);
    assert(
      responce_matchable.type() == moving_.type() &&
      "CorrespondenceFinderMatchablesKDTree::_findPointAssociation|correspondence types mismatch");


    //removeeeee
    // ia check appearence of association
    // const auto descriptor_distance =
    //   cv::norm(responce_matchable.descriptor(), moving_.descriptor(), cv::NORM_HAMMING);

    // ia if appearence is not good return invalid
    // if (descriptor_distance >= param_max_hamming_distance_points.value()) {
    //   ++_stats.non_associated;
    //    return c;
    //  }

    c.fixed_idx  = responce_idx;
    c.moving_idx = moving_idx_;
    // c.response   = descriptor_distance;
    c.response   = 0.; //todo assign a value to this variable since it describes how much the two features are similar
    // namely the distance can be a good choice


    ++_stats.associated_points;
    return c;
  }

  srrg2_core::Correspondence CorrespondenceFinderMatchablesKDTree::_findLineAssociation(
    const size_t& moving_idx_,
    CustomMatchablef& moving_,
    CustomMatchablef& moving_transformed_) {
    Correspondence c;

    KdTreeTypeLines::VectorTD responce_coords = KdTreeTypeLines::VectorTD::Zero();
    int responce_idx                          = -1;

    // ia transform the moved matchable into tree compliant coords
    KdTreeTypeLines::VectorTD moving_tree_coords = KdTreeTypeLines::VectorTD::Zero();
    moving_tree_coords.head<3>() =
      (moving_transformed_.direction()).cross(moving_transformed_.origin());
    moving_tree_coords.tail<3>() = moving_transformed_.direction() * _line_direction_tree_weight;

    // ia query the tree
    //    const float responce_distance =
    _kd_tree_lines->findNeighbor(
      responce_coords, responce_idx, moving_tree_coords, param_max_leaf_range_points.value());

    // ia if kdtree fails return invalid correspondence
    if (responce_idx < 0) {
      ++_stats.non_associated;
      return c;
    }

    // ia get the matchable from the fixed
    const CustomMatchablef& responce_matchable = _fixed->at(_index_map_lines[responce_idx]);
    assert(
      responce_matchable.type() == moving_.type() &&
      "CorrespondenceFinderMatchablesKDTree::_findLineAssociation|correspondence types mismatch");

    // ia check appearence of association
    //const auto descriptor_distance =
    //  cv::norm(responce_matchable.descriptor(), moving_.descriptor(), cv::NORM_HAMMING);

    // ia if appearence is not good return invalid
    // if (descriptor_distance >= param_max_hamming_distance_lines.value()) {
    //   ++_stats.non_associated;
    //  return c;
    // }

    c.fixed_idx  = responce_idx;
    c.moving_idx = moving_idx_;
    //c.response   = descriptor_distance;
    c.response   = 0.; //todo assign a value to this variable since it describes how much the two features are similar
    // namely the distance can be a good choice

    ++_stats.associated_lines;
    return c;
  }

  srrg2_core::Correspondence CorrespondenceFinderMatchablesKDTree::_findPlaneAssociation(
    const size_t& moving_idx_,
    CustomMatchablef& moving_,
    CustomMatchablef& moving_transformed_) {
    Correspondence c;

    KdTreeTypePlanes::VectorTD responce_coords    = KdTreeTypePlanes::VectorTD::Zero();
    KdTreeTypePlanes::VectorTD moving_tree_coords = KdTreeTypePlanes::VectorTD::Zero();
    moving_tree_coords.head<3>() = moving_transformed_.direction() * _plane_direction_tree_weight;
    moving_tree_coords(3) = moving_transformed_.direction().dot(moving_transformed_.origin());

    int responce_idx = -1;

    // ia query the tree
    //    const float responce_distance =
    _kd_tree_planes->findNeighbor(
      responce_coords, responce_idx, moving_tree_coords, param_max_leaf_range_points.value());

    // ia if kdtree fails return invalid correspondence
    if (responce_idx < 0) {
      ++_stats.non_associated;
      return c;
    }

    // ia get the matchable from the fixed
    const CustomMatchablef& responce_matchable = _fixed->at(_index_map_planes[responce_idx]);
    assert(
      responce_matchable.type() == moving_.type() &&
      "CorrespondenceFinderMatchablesKDTree::_findPlaneAssociation|correspondence types mismatch");

    // ia check consistency of the association
    float dist   = responce_matchable.direction().dot(moving_.direction());
    float dist_d = std::fabs(responce_matchable.direction().dot(responce_matchable.origin()) -
                             (moving_.direction()).dot(moving_.origin()));
    if (dist < 0.8 || dist_d > 0.1) {
      ++_stats.non_associated;
      return c;
    }

    c.fixed_idx  = responce_idx;
    c.moving_idx = moving_idx_;
    c.response   = dist; // ia is this good??

    ++_stats.associated_planes;
    return c;
  }

}
