#pragma once
#include <srrg_config/param_macros.h>
#include <srrg_property/property.h>

#include "./CorrespondenceFinderMatchables.hpp"

namespace Loam{

  typedef struct dataAssociationEntry_tag{
    int fixed_index;
    int moving_index;
    double confidence_score;
    dataAssociationEntry_tag( int t_1, int t_2, double t_3):
      fixed_index( t_1),
      moving_index( t_2),
      confidence_score( t_3)
    {}
    dataAssociationEntry_tag():
      fixed_index(-1),
      moving_index(-1),
      confidence_score(-1)
    {}
  } dataAssociationEntry;


  class CorrespondenceFinderMatchablesBruteForce : public CorrespondenceFinderMatchables {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // ia just in case
    using BaseType = CorrespondenceFinderMatchables;
    using ThisType = CorrespondenceFinderMatchablesBruteForce;
    using Scalar   = BaseType::TransformType::Scalar;

  public:

    //! @brief ctor
    CorrespondenceFinderMatchablesBruteForce() : BaseType() {
    }

    //! @brief dtor
    virtual ~CorrespondenceFinderMatchablesBruteForce() {
    }

    void reset() final;

    //! @brief override of BaseType::compute (non overridable).
    void compute() final;

  protected:

    void  associateMatchables();

    std::vector<dataAssociationEntry> findCandidates(
        const int t_moving_matchable_index, const MatchableBase::Type t_type);

    void insertOrderedDataAssociation(
      const dataAssociationEntry &t_matchablesComparison,
      std::vector<dataAssociationEntry> &t_candidates);

    dataAssociationEntry compareMatchables(
        const int t_fixed_index,
        const int t_moving_index,
        const MatchableBase::Type t_type);

    std::vector< int> getPossibleCandidateIndexes(
        const int t_moving_index , const MatchableBase::Type t_type);

    std::vector<dataAssociationEntry> chooseBestAssociations(
        std::vector< std::vector< dataAssociationEntry>> &t_matrix);

    void removeTakenAssociations(
        const dataAssociationEntry &t_takenAssociation,
        std::vector< std::vector< dataAssociationEntry>> &t_matrix);

    dataAssociationEntry chooseMaxScoreAssociation(
        const std::vector< std::vector< dataAssociationEntry>> &t_matrix);

    bool associationsAllTaken(
        const std::vector< std::vector< dataAssociationEntry>> &t_matrix);

    bool _is_initialized = false;
 };

  using CorrespondenceFinderMatchablesBruteForcePtr= std::shared_ptr<CorrespondenceFinderMatchablesBruteForce>;

}
