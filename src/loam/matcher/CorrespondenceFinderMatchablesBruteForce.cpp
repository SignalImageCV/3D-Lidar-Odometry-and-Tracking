#include "./CorrespondenceFinderMatchablesBruteForce.hpp"

namespace Loam{

  using namespace srrg2_core;

  void CorrespondenceFinderMatchablesBruteForce::reset() {
    BaseType::reset();

    _is_initialized = false;

    if (!_fixed) {
      throw std::runtime_error("CorrespondenceFinderMatchablesKDTree::reset|error, fixed not set");
    }
    
  }

  void CorrespondenceFinderMatchablesBruteForce::compute() {
    _correspondences->reserve(_moving->size());

    associateMatchables();
  }

  void  CorrespondenceFinderMatchablesBruteForce::associateMatchables(){


    std::vector< std::vector< dataAssociationEntry> > points_associations;
    points_associations.reserve( _moving->size());
    std::vector< std::vector< dataAssociationEntry> > lines_associations;
    lines_associations.reserve( _moving->size());
    std::vector< std::vector< dataAssociationEntry> > planes_associations;
    planes_associations.reserve( _moving->size());



    for (size_t k = 0; k < _moving->size(); ++k) {
      CustomMatchablef& moving_m = _moving->at(k);


      switch (moving_m.type()) {
        case MatchableBase::Type::Point: {
          points_associations.push_back( findCandidates( k, MatchableBase::Type::Point ));
          break;
        }
        case MatchableBase::Type::Line: {
          lines_associations.push_back( findCandidates( k, MatchableBase::Type::Line));
          break;
        }
        case MatchableBase::Type::Plane: {
          planes_associations.push_back( findCandidates( k, MatchableBase::Type::Plane ));
          break;
        }
        default:
          std::cerr << "CorrespondenceFinderMatchablesKDTree::compute|unknown matchable type [" +
          std::to_string((int) moving_m.type()) + "]";
      }
    }

//    std::cout<< "size points"<< points_associations.size()<<"\n"; 
//    std::cout<< "size lines"<< lines_associations.size()<<"\n"; 
//    std::cout<< "size planes"<< planes_associations.size()<<"\n"; 

    std::vector<dataAssociationEntry> pointAssociations = chooseBestAssociations( points_associations);
    std::vector<dataAssociationEntry> lineAssociations = chooseBestAssociations( lines_associations);
    std::vector<dataAssociationEntry> planeAssociations = chooseBestAssociations( planes_associations);

    for ( auto & assoc : pointAssociations ){
      Correspondence c;
      c.fixed_idx = assoc.fixed_index;
      c.moving_idx= assoc.moving_index;
//      std::cout<< "Confidence Score :"<< assoc.confidence_score<< "\n"; 
      _correspondences->emplace_back(c);
      ++_stats.associated_points;
    }
    for ( auto & assoc : lineAssociations ){
      Correspondence c;
      c.fixed_idx = assoc.fixed_index;
      c.moving_idx= assoc.moving_index;
//      std::cout<< "Confidence Score :"<< assoc.confidence_score<< "\n"; 
      _correspondences->emplace_back(c);
      ++_stats.associated_lines;
    }
    for ( auto & assoc : planeAssociations ){
      Correspondence c;
      c.fixed_idx = assoc.fixed_index;
      c.moving_idx= assoc.moving_index;
//      std::cout<< "Confidence Score :"<< assoc.confidence_score<< "\n"; 
      _correspondences->emplace_back(c);
      ++_stats.associated_planes;
    }
  }

    std::vector<dataAssociationEntry> CorrespondenceFinderMatchablesBruteForce::findCandidates
    (const int t_moving_matchable_index, const MatchableBase::Type t_type){

      std::vector< dataAssociationEntry> candidates;
      candidates.reserve( _fixed->size());
      std::vector< int> good_indexes =
        getPossibleCandidateIndexes( t_moving_matchable_index, t_type);

      for( auto i : good_indexes){
        insertOrderedDataAssociation(compareMatchables(i, t_moving_matchable_index, t_type),
            candidates);
      }
      return candidates;
    }



    void CorrespondenceFinderMatchablesBruteForce::insertOrderedDataAssociation(
        const dataAssociationEntry &t_matchablesComparison,
        std::vector<dataAssociationEntry> &t_candidates){

      if ( t_matchablesComparison.confidence_score >= 0){
        bool inserted = false;
        int i = 0;
        while ( !inserted && i < t_candidates.size() ){
          if ( t_candidates.at(i).confidence_score <=
              t_matchablesComparison.confidence_score){
            t_candidates.insert( t_candidates.begin()+i, t_matchablesComparison);
            inserted = true;
          }
          ++i;
        }
        if (!inserted){
          t_candidates.insert( t_candidates.end(), t_matchablesComparison);
        }
      }
    }


    dataAssociationEntry CorrespondenceFinderMatchablesBruteForce::compareMatchables(
        const int t_fixed_index,
        const int t_moving_index,
        const MatchableBase::Type t_type){

      dataAssociationEntry result;
      result.fixed_index = t_fixed_index;
      result.moving_index = t_moving_index;

      CustomMatchablef& fixed_m = _fixed->at(t_fixed_index);
      CustomMatchablef& moving_m = _moving->at(t_moving_index);
      CustomMatchablef moving_m_transformed = moving_m.transform(_estimate);

      const Vector3f fixed_origin_normalized = fixed_m.origin().normalized();
      const Vector3f moving_origin_normalized = moving_m_transformed.origin().normalized();

      const float  cartesian_distance =  (fixed_m.origin() - moving_m_transformed.origin()).norm();
      const float  cartesian_distance_pseudonormalized =  cartesian_distance / moving_m_transformed.origin().norm();

//      std::cout<<" cartesian_distance "<< cartesian_distance<< "\n";
//      std::cout<<" cartesian_distance_pseudonormalized "<< cartesian_distance_pseudonormalized<< "\n";

      switch (t_type) {
        case MatchableBase::Type::Point: {
          result.confidence_score = 100 - (cartesian_distance_pseudonormalized * 30);
          break;
        }
        case MatchableBase::Type::Line: {
          const float absolute_orientation_difference= 1. - fixed_m.direction().dot(moving_m.direction());
          const float relative_orientation_difference = std::fabs(fixed_m.direction().dot(fixed_origin_normalized) -
            (moving_m_transformed.direction()).dot(moving_origin_normalized));

//          std::cout<<" absolute_orientation_difference "<< absolute_orientation_difference << "\n";

      //    std::cout<<" fixed_m direction"<< fixed_m.direction() << "\n";
//          std::cout<<" fixed_m origin"<< fixed_m.origin() << "\n";
//          std::cout<<" moving_m origin"<< moving_m.origin() << "\n";
//          std::cout<<" moving_m direction"<< moving_m.direction() << "\n";
//
//          std::cout<<" relative_orientation_arg_1 "<< (fixed_m.direction()).dot(fixed_m.origin()) << "\n";
//          std::cout<<" relative_orientation_arg_2 "<< (moving_m_transformed.direction()).dot(moving_m_transformed.origin())<< "\n";
//          std::cout<<" relative_orientation_difference "<< relative_orientation_difference << "\n";
//
          result.confidence_score = 100 - cartesian_distance_pseudonormalized * 30 -
            absolute_orientation_difference *110 - relative_orientation_difference *110;
          break;
        }
        case MatchableBase::Type::Plane: {
          const float absolute_orientation_difference= fixed_m.direction().dot(moving_m.direction());
          const float relative_orientation_difference = std::fabs(fixed_m.direction().dot(fixed_origin_normalized) -
            (moving_m_transformed.direction()).dot(moving_origin_normalized));

//          std::cout<<" absolute_orientation_difference "<< absolute_orientation_difference << "\n";
//
//          std::cout<<" fixed_m direction"<< fixed_m.direction() << "\n";
//          std::cout<<" fixed_m origin"<< fixed_m.origin() << "\n";
//          std::cout<<" moving_m origin"<< moving_m.origin() << "\n";
//          std::cout<<" moving_m direction"<< moving_m.direction() << "\n";
//
//          std::cout<<" relative_orientation_arg_1 "<< (fixed_m.direction()).dot(fixed_m.origin()) << "\n";
//          std::cout<<" relative_orientation_arg_2 "<< (moving_m_transformed.direction()).dot(moving_m_transformed.origin())<< "\n";
//          std::cout<<" relative_orientation_difference "<< relative_orientation_difference << "\n";

          result.confidence_score = 100 - cartesian_distance_pseudonormalized * 30 -
            absolute_orientation_difference *110 - relative_orientation_difference *110;
          break;
        }
        default: {}
        //todo
        // normalize the origin of the points before taking the dot product

      }

//      std::cout<<" confidence score  of fixed "<< result.fixed_index << " with moving "<<result.moving_index<< " and score "<< result.confidence_score << " \n"; 

      return result;
    }



    std::vector< int> CorrespondenceFinderMatchablesBruteForce::getPossibleCandidateIndexes(
        const int t_moving_index , const MatchableBase::Type t_type){

      std::vector< int> indexes;
      indexes.reserve(_fixed->size());

      for (size_t k = 0; k < _fixed->size(); ++k) {
        const CustomMatchablef & m = _fixed->at(k);
        if (m.type() == t_type) {
          indexes.push_back(k);
        }
      }
      return indexes;
    }
    


    std::vector<dataAssociationEntry> CorrespondenceFinderMatchablesBruteForce::chooseBestAssociations(
        std::vector< std::vector< dataAssociationEntry>> &t_matrix){

      std::vector<dataAssociationEntry> associations;
      associations.reserve( _moving->size());
      while ( !associationsAllTaken(t_matrix) ){

        dataAssociationEntry curr_bestAssociation =
          chooseMaxScoreAssociation( t_matrix);
        removeTakenAssociations( curr_bestAssociation, t_matrix);
        associations.push_back( curr_bestAssociation);
      }
      return associations;
    }

    void CorrespondenceFinderMatchablesBruteForce::removeTakenAssociations(
        const dataAssociationEntry &t_takenAssociation,
        std::vector< std::vector< dataAssociationEntry>> &t_matrix){

      for (auto &vec : t_matrix){
        vec.erase
          ( std::remove_if
            (vec.begin(),
            vec.end(),
              [t_takenAssociation](dataAssociationEntry dA) -> bool{
                return dA.fixed_index == t_takenAssociation.fixed_index ||
                dA.moving_index == t_takenAssociation.moving_index;
           }
           ), vec.end() );
      }
    }

    dataAssociationEntry CorrespondenceFinderMatchablesBruteForce::chooseMaxScoreAssociation(
        const std::vector< std::vector< dataAssociationEntry>> &t_matrix){

      dataAssociationEntry bestAssociation;
      bestAssociation.confidence_score = -100;
      for (auto vec : t_matrix){
        if ( !vec.empty()){
          dataAssociationEntry currAssociation = vec.front();
          if ( currAssociation.confidence_score >
              bestAssociation.confidence_score){
            bestAssociation = currAssociation;
          }
        }
      }
      return bestAssociation;
    }


    bool CorrespondenceFinderMatchablesBruteForce::associationsAllTaken(
        const std::vector< std::vector< dataAssociationEntry>> &t_matrix){
      bool allTaken = true;
      for (auto vec : t_matrix){
        if ( !vec.empty()){
          allTaken = false;
        }
      }
      return allTaken;
    }
}
