#pragma once
#include <srrg_config/param_macros.h>
#include <srrg_property/property.h>

#include <srrg_data_structures/kd_tree.hpp>

#include "./CorrespondenceFinderMatchables.hpp"

namespace Loam{

  //! @brief derived class that uses kd trees to perform data association
  class CorrespondenceFinderMatchablesKDTree : public CorrespondenceFinderMatchables {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // ia just in case
    using BaseType = CorrespondenceFinderMatchables;
    using ThisType = CorrespondenceFinderMatchablesKDTree;
    using Scalar   = BaseType::TransformType::Scalar;

    // ia kd tree specifications
    static constexpr size_t KdTreeDimension_Points = 3;
    static constexpr size_t KdTreeDimension_Lines  = 6;
    static constexpr size_t KdTreeDimension_Planes = 4;

    using KdTreeTypePoints = srrg2_core::KDTree<Scalar, KdTreeDimension_Points>;
    using KdTreeTypeLines  = srrg2_core::KDTree<Scalar, KdTreeDimension_Lines>;
    using KdTreeTypePlanes = srrg2_core::KDTree<Scalar, KdTreeDimension_Planes>;

  public:
    //! @brief paramters
    PARAM(srrg2_core::PropertyUnsignedInt,
          min_leaf_points,
          "minimum number of points in a leaf of the KDTree [points association]",
          1,
          0);
    PARAM(srrg2_core::PropertyFloat,
          max_leaf_range_points,
          "maximum leaf range in meters for the KDTree [points association]",
          1e-4f,
          0);
    PARAM(srrg2_core::PropertyUnsignedInt,
          max_hamming_distance_points,
          "maximum distance to associate two points",
          25,
          0);
    PARAM(srrg2_core::PropertyUnsignedInt,
          max_hamming_distance_lines,
          "maximum distance to associate two lines",
          15,
          0);

  public:
    //! @brief ctor
    CorrespondenceFinderMatchablesKDTree() : BaseType() {
    }

    //! @brief dtor
    virtual ~CorrespondenceFinderMatchablesKDTree() {
    }

    //! @brief resets the kdtrees and populates them
    void reset() final;

    //! @brief override of BaseType::compute (non overridable).
    //! in this case we have to populate the kdtrees from the fixed and then,
    //! for each matchable in the moving, query the trees according to the type of primitive.
    void compute() final;

  protected:
    //! @brief aux function to associate points with points - returns a correspondence
    //! if correspondence is wrong returns it with indices -1 -1
    srrg2_core::Correspondence
    _findPointAssociation(const size_t& moving_idx_,
                          CustomMatchablef& moving_,
                          CustomMatchablef& moving_transformed_);

    //! @brief aux function to associate lines with lines - returns a correspondence
    //! if correspondence is wrong returns it with indices -1 -1
    srrg2_core::Correspondence
    _findLineAssociation(const size_t& moving_idx_,
                         CustomMatchablef& moving_,
                         CustomMatchablef& moving_transformed_);

    //! @brief aux function to associate planes with planes - returns a correspondence
    //! if correspondence is wrong returns it with indices -1 -1
    srrg2_core::Correspondence
    _findPlaneAssociation(const size_t& moving_idx_,
                          CustomMatchablef& moving_,
                          CustomMatchablef& moving_transformed_);

  protected:
    // ia unique pointers of the 3 kd trees
    std::unique_ptr<KdTreeTypePoints> _kd_tree_points = nullptr;
    std::unique_ptr<KdTreeTypeLines> _kd_tree_lines   = nullptr;
    std::unique_ptr<KdTreeTypePlanes> _kd_tree_planes = nullptr;

    // ia stupid flag
    bool _is_initialized = false;

    // ia since we have 3 different kdtrees we must have a map that gives you the index in the fixed
    // given the tree index
    std::vector<size_t> _index_map_points;
    std::vector<size_t> _index_map_lines;
    std::vector<size_t> _index_map_planes;

    // ia maybe you want to scale this to model the importance of this component in the tree
    const float _line_direction_tree_weight  = 1.0f;
    const float _plane_direction_tree_weight = 10.0f;
  };

  using CorrespondenceFinderMatchablesKDTreePtr = std::shared_ptr<CorrespondenceFinderMatchablesKDTree>;

}
