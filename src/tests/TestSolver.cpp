#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/system_utils.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/shell_colors.h>
#include <srrg_messages/instances.h>

#include "loam/CustomMeasurementAdaptor.hpp"
#include "loam/instances.h"
#include "loam/matcher/CorrespondenceFinderMatchablesKDtree.hpp"
#include "loam/DatasetManager.hpp"
#include "loam/Visualizer.hpp"


#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/SE3/instances.h>



using namespace srrg2_core;
using namespace srrg2_solver;
using namespace srrg2_core_ros;
using namespace srrg2_qgl_viewport;
using namespace Loam;


const char* banner[] = {
    "dynamic executor",
      0
};

void processSolve(
    ViewerCanvasPtr canvas_1,
    ViewerCanvasPtr canvas_2,
    ViewerCanvasPtr canvas_3,
    const  string & filename,
    const sphericalImage_params t_params);


PointNormalColor3fVectorCloud drawLine( Vector3f p1,  Vector3f p2, Vector3f color );



int main( int argc, char** argv){

  messages_registerTypes();
  loam_registerTypes();
  SE3_registerTypes();
  srrgInit( argc, argv, "hi");

  ParseCommandLine cmd_line(argv,banner);

  ArgumentString  dataset (&cmd_line, "d", "dataset", "path to dataset" , "");
  ArgumentString  num_vertical_rings(&cmd_line, "vr", "num_vertical_rings", "num of vertical rings" , "");
  ArgumentString  num_points_ring(&cmd_line, "hr", "num_points_ring", "num of horizontal rings (slices)" , "");
  ArgumentString  epsilon_t(&cmd_line, "et", "epsilon_t", "min num of times points have to fall in the base circle to be considered vertical " , "");
  ArgumentString  epsilon_r(&cmd_line, "er", "epsilon_r", "min radius of the circle in which vertical points have to fall" , "");
  ArgumentString  depth_differential_threshold(&cmd_line, "dd", "depth_differential_threshold", "max difference between depths in near points to be considered valid points" , "");
  ArgumentString  min_neighboors_for_normal(&cmd_line, "nn", "min_neighboors_for_normal", "min num of points to use to compute the normals" , "");
  ArgumentString  epsilon_c(&cmd_line, "ec", "epsilon_c", "min num of points that forma a cluster" , "");
  ArgumentString  epsilon_d(&cmd_line, "ed", "epsilon_d", "min cartesian distance between points of the same cluster" , ""); //currently unused
  ArgumentString  epsilon_n(&cmd_line, "en", "epsilon_n", "min distance between directions of normals of points in the same cluster" , "");
  ArgumentString  epsilon_l(&cmd_line, "el", "epsilon_l", "min number that descibes the eigenvalue constraint of a line" , "");
  ArgumentString  epsilon_dl(&cmd_line,"edl", "epsilon_dl", "min number of the cumulative cartesian error of the matchable fitted to a line" , "");
  ArgumentString  epsilon_p(&cmd_line, "ep", "epsilon_p", "min number that descibes the eigenvalue constraint of a plane" , "");
  ArgumentString  epsilon_dp(&cmd_line,"edp", "epsilon_dp", "min number of the cumulative cartesian error of the matchable fitted to a plane" , "");
  cmd_line.parse();

  const sphericalImage_params params(
    std::stoi( num_vertical_rings.value()), 
    std::stoi( num_points_ring.value()),
    std::stoi( epsilon_t.value()),
    std::stof( epsilon_r.value()),
    std::stof( depth_differential_threshold.value()),
    std::stoi( min_neighboors_for_normal.value()),
    std::stoi( epsilon_c.value()),
    std::stof( epsilon_d.value()),
    std::stof( epsilon_n.value()),
    std::stof( epsilon_l.value()),
    std::stof( epsilon_dl.value()),
    std::stof( epsilon_p.value()),
    std::stof( epsilon_dp.value())
  );

  QApplication qapp(argc, argv);
  ViewerCoreSharedQGL viewer(argc, argv, &qapp);
  ViewerCanvasPtr canvas1 = viewer.getCanvas("matchables");
  ViewerCanvasPtr canvas2 = viewer.getCanvas("clusters");
  ViewerCanvasPtr canvas3 = viewer.getCanvas("associations");
  std::thread processing_thread1(
      processSolve,
      canvas1,
      canvas2,
      canvas3,
      dataset.value(),
      params
      );

  viewer.startViewerServer();
  processing_thread1.join();
  return 0;
}


PointNormalColor3fVectorCloud drawLine( Vector3f p1,  Vector3f p2, Vector3f color ){

  const float dist_points = (p1 - p2).norm();
  const float epsilon = 0.1;
  const int num_points = static_cast <int> ( dist_points / epsilon);
  PointNormalColor3fVectorCloud linePoints;
  linePoints.resize( num_points);

  const Vector3f line_vec= p2 - p1; 
  for ( float  i = 0; i< num_points; ++i){
    const float curr_advancement= ( i / num_points);
    const Vector3f from_p1 =  line_vec * curr_advancement; 
    const Vector3f curr_coord = p1 + from_p1;
    linePoints[i].coordinates() = curr_coord;
    linePoints[i].color() = color;
 }
  return linePoints;
}




void processSolve(
    ViewerCanvasPtr canvas_1,
    ViewerCanvasPtr canvas_2,
    ViewerCanvasPtr canvas_3,
    const  string & filename,
    const sphericalImage_params t_params){

  DatasetManager dM(  filename);
  BaseSensorMessagePtr cloudPtr1 = dM.readPointerToMessageFromDataset();
  BaseSensorMessagePtr cloudPtr2 = dM.readPointerToMessageFromDataset();
  const float x_max= 0;
  const float y_max= 0;
  const float z_max= 0;
  const float rx_max= 0;
  const float ry_max= 0;
  const float rz_max= 0.3;

  const float points_size = 1.5;

  while( ViewerCoreSharedQGL::isRunning()) {
    const float x = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / x_max));
    const float y = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / y_max));
    const float z = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / z_max));
    const float rx = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / rx_max));
    const float ry = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / ry_max));
    const float rz = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / rz_max));

    Vector6f pose;
    pose << x, y, z, rx, ry, rz;
    const Isometry3f rotoTransl = srrg2_core::geometry3d::v2t(pose);


    CustomMatchablefVectorData  matchables_1;
    CustomMatchablefVectorData  matchables_2;

    CustomMeasurementAdaptorPtr measurementAdaptor =
      CustomMeasurementAdaptorPtr(new CustomMeasurementAdaptor);
    measurementAdaptor->setMyParams(t_params);

    measurementAdaptor->setDest( &matchables_1);
    measurementAdaptor->setMeasurement( cloudPtr1 );
    measurementAdaptor->compute();

    PointNormalColor3fVectorCloud  features_cloud_1 = measurementAdaptor->drawMatchables();
    PointNormalColor3fVectorCloud  clusters_cloud_1 = measurementAdaptor->drawClusters();
    PointNormalColor3fVectorCloud  clusters_cloud_1_corrected= measurementAdaptor->drawClusters();
    measurementAdaptor->reset();

    measurementAdaptor->setDest( &matchables_2);
    measurementAdaptor->setMeasurement( cloudPtr2);
    measurementAdaptor->compute();

    PointNormalColor3fVectorCloud  features_cloud_2 = measurementAdaptor->drawMatchables();
    PointNormalColor3fVectorCloud  clusters_cloud_2 = measurementAdaptor->drawClusters();


    const Isometry3f T = Isometry3f::Identity() * rotoTransl;
    for ( auto & m: matchables_2){
      m.transformInPlace(T);
    }
    features_cloud_2.transformInPlace( T);
    clusters_cloud_2.transformInPlace( T);

    CorrespondenceVector correspondances;

    CorrespondenceFinderMatchablesKDTreePtr correspondenceFinder =
      CorrespondenceFinderMatchablesKDTreePtr( new CorrespondenceFinderMatchablesKDTree);


    correspondenceFinder->setCorrespondences( &correspondances);
    correspondenceFinder->setEstimate(Isometry3f::Identity());
    correspondenceFinder->setFixed(&matchables_1 );
    correspondenceFinder->setMoving(&matchables_2);
    correspondenceFinder->reset();
    correspondenceFinder->compute();

    std::cout << "correspondances || " << correspondenceFinder->stats() << std::endl;



    using FactorType          = SE3Matchable2MatchableErrorFactorNoInfo;
    using FactorBaseType      = Factor_<FactorType::VariableType::BaseVariableType>;

    const int num_iterations = 10;
    std::vector<FactorBaseType*> factors;
    factors.reserve(correspondances.size());


    for ( auto & c : correspondances){
      FactorType* factor = new FactorType();
      factor->bindFixed(&matchables_1[c.fixed_idx]);
      factor->bindMoving(&matchables_2[c.moving_idx]);
      factors.emplace_back(factor);
    }

    const Isometry3f guess = Isometry3f::Identity();

    SolverDefault_<VariableSE3EulerLeftAD> solver;
    solver.param_max_iterations.pushBack(num_iterations);
    solver.param_termination_criteria.setValue(nullptr);
    solver.clearFactorIterators(); // ia JIC
    solver.addFactorContainer(factors);
    solver.setEstimate(guess);
    solver.compute();
    const auto& stats      = solver.iterationStats();
    const auto& final_chi2 = stats.back().chi_inliers;


    std::cerr << stats << std::endl;
    std::cerr << " final chi : "<< final_chi2 << std::endl;

    const auto& estimated_T = solver.estimate();
    std::cerr << "GT\n" << FG_GREEN(T.matrix()) << std::endl;
    std::cerr << "estimated\n" << FG_YELLOW(estimated_T.matrix()) << std::endl;

    // const auto diff_T      = estimated_T.inverse() * T;
    // const auto diff_vector = geometry3d::t2tnq(diff_T);


    for (size_t i = 0; i < factors.size(); ++i) {
      delete factors[i];
    }

    for( auto & p: clusters_cloud_1){
      p.color() = Vector3f( 1.,0.,0.);
    }
    for( auto & m: features_cloud_1 ){
      m.color() = Vector3f( 1.,0.,0.);
    }
    for( auto & p: clusters_cloud_2){
      p.color() = Vector3f( 0.,0.,1.);
    }
    for( auto & m: features_cloud_2 ){
      m.color() = Vector3f( 0.,0.,1.);
    }

    clusters_cloud_1_corrected.transformInPlace( estimated_T);
    for( auto & p: clusters_cloud_1_corrected){
      p.color() = Vector3f( 0.,1.,0.);
    }

    canvas_1->pushPointSize();
    canvas_1->setPointSize(points_size);
    canvas_1->putPoints( features_cloud_1);
    canvas_1->putPoints( features_cloud_2);
    canvas_1->flush();

    canvas_2->pushPointSize();
    canvas_2->setPointSize(points_size);
    canvas_2->putPoints( clusters_cloud_1);
    canvas_2->putPoints( clusters_cloud_2);
    canvas_2->putPoints( clusters_cloud_1_corrected);
    canvas_2->flush();

    canvas_3->pushPointSize();
    canvas_3->setPointSize(points_size);
    canvas_3->putPoints( clusters_cloud_1);
    canvas_3->putPoints( clusters_cloud_2);
    canvas_3->popAttribute();
    canvas_3->setPointSize(points_size*4);

    for ( auto & c: correspondances){
      const PointNormalColor3fVectorCloud  linePoints =
        drawLine(
            matchables_1[c.fixed_idx].origin(),
            matchables_2[c.moving_idx].origin(),
            Vector3f(0.,0.,0. ));
      canvas_3->putPoints( linePoints);
    }
    canvas_3->popAttribute();
    canvas_3->flush();

    std::getchar();
  }

}

