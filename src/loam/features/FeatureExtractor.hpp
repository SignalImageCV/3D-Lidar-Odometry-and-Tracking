#pragma once
#include "../interfaces/FeatureExtractorInterface.hpp"
#include "../ScanPoint.hpp"

namespace Loam{

  class FeatureExtractor: FeatureExtractorInterface{
    protected:
      Point3fVectorCloud  m_cloud;

    public:
      FeatureExtractor() = default;

      FeatureExtractor( Point3fVectorCloud & t_cloud );

      ~FeatureExtractor() = default;

      vector<Matchable> extractFeatures();

      //Collections of functions that given a cloud of points in a vertical slice
      // (called scan plane) e.g. produced in a single instant from
      // a lidar, computes the smoothness c of every one
      // it updates the c value inside the struct of each point
      static void computeSingleSmoothnessPaper(const std::vector<ScanPoint> & other_points, ScanPoint & point);
      static void computeSmoothnessPaper( std::vector<ScanPoint> & points);
      static void computeSingleSmoothnessMine(const std::vector<ScanPoint> & other_points, ScanPoint & point);
      static void computeSmoothnessMine( std::vector<ScanPoint> & points);

      static float computeSmoothness( const  srrg2_core::Point3fVectorCloud & cloud, const int index_point);

      static ScanPoint findMaxSmoothnessPoint( const  std::vector<ScanPoint> & points);
      static ScanPoint findMinSmoothnessPoint( const  std::vector<ScanPoint> & points);

      static std::list<ScanPoint> sortForDecreasingSmoothness(const  std::vector<ScanPoint> & points);
      static std::list<ScanPoint> sortForIncreasingSmoothness(const  std::vector<ScanPoint> & points);
      static std::vector<std::vector<ScanPoint>>  divideInSectors( const int num_sectors, const  std::vector<ScanPoint> & points);

  };
}

