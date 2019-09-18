#pragma once
#include <srrg_config/property_configurable.h>
#include <srrg_matchable/matchable.h>
#include <srrg_pcl/point_normal_curvature_color.h>
#include <srrg_messages/instances.h>

namespace Loam{
  using namespace srrg2_core;


  template <typename Scalar_>
  class CustomMatchable_ : public Matchable_<Scalar_>{
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

      using Scalar = Scalar_;

      using ThisType            = CustomMatchable_<Scalar_>;
      using MatchableType       = Matchable_<Scalar_>;
  //    using ExtentType          = PointNormalColor3f;

      using VectorType         = Vector3_<Scalar>;
      using MatrixType         = Matrix3_<Scalar>;
      static constexpr int Dim = VectorType::RowsAtCompileTime + // ia origin
                               MatrixType::RowsAtCompileTime + // ia direction
                               1;                              // ia type
      using FullVectorType = Vector_<Scalar, Dim>;

 //     CustomMatchable_() : MatchableType() { }

      using CustomMatchablef = CustomMatchable_<float>;
      using CustomMatchabled = CustomMatchable_<double>;

   };

   //! @brief containers
   template <typename Scalar_>
   using CustomMatchableVector_ =
      std::vector<CustomMatchable_<Scalar_>,
      Eigen::aligned_allocator<CustomMatchable_<Scalar_>>>;

   using CustomMatchablefVector = CustomMatchableVector_<float>;
   using CustomMatchabledVector = CustomMatchableVector_<double>;

   using CustomMatchabledVectorData = VectorData_<CustomMatchabledVector>;
   using CustomMatchablefVectorData = VectorData_<CustomMatchablefVector>;

}




