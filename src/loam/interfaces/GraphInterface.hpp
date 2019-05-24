#pragma once

namespace Loam{

  typedef struct node_tag{

    std::vector<scanPoint> scanPoints_robotFrame;
    std::vector<feature> features;

    node_tag( state t_1,
        std::vector<scanPoint> t_2,
        std::vector<line> t_3):
      q( t_1),
      scanPoints_robotFrame( t_2),
      lines( t_3)
    {}
    node_tag():
      q()
    {
      scanPoints_robotFrame = {};
      lines = {};
    }
  } node;

  typedef struct edge_tag{
    Eigen::Vector3d delta_x;
    std::vector<dataAssociation> associations;

    edge_tag(
        Eigen::Vector3d t_1,
        std::vector<dataAssociation> t_2):
      delta_x( t_1),
      associations( t_2)
    {}
    edge_tag( 
        double t_x,
        double t_y,
        double t_theta,
        std::vector<dataAssociation> t_associations):
      delta_x( t_x, t_y, t_theta),
      associations( t_associations)
    {}
    edge_tag(){
      delta_x = Eigen::Vector3d::Zero();
      associations = {};
    }
  } edge;


  class GraphInterface {
    protected:
      int m_maxPointsNum;
      std::vector<node> m_nodes;
      std::vector<edge> m_edges;

    public:

      GraphInterface() = default;

      //TODO adapt structures and also the construct and check all the graph methods to mantain generality of purpose
      GraphInterface(  const int t_maxPointsNum ):
        m_maxPointsNum( t_maxPointsNum)
    {};

      virtual ~GraphInterface() = default;

      virtual void insertNode( const node &t_node) =0;

      virtual void insertEdge( const edge &t_edge) =0;

      virtual edge getEdge( const int t_index ) =0;

      virtual node getNode( const int t_index ) =0;

      virtual edge& getEdgeRef( const int t_index ) =0;

      virtual node& getNodeRef( const int t_index ) =0;

      virtual void changeNode(const int t_index,const node &t_node) =0;

      virtual void changeEdge(const int t_index,const edge &t_edge) =0;

      virtual std::vector<node> getNodes() =0;

      virtual std::vector<edge> getEdges() =0; 

      virtual void updateNodeStates() =0;

      virtual std::vector<trail> findTrails(const int t_nodeFromIndex,
          const int t_nodeToIndex ) =0;

      virtual void addOldTrails( std::vector<trail> &oldTrails,
          std::map< int, trail> &t_unmatchedTrails) =0;

      virtual std::map<int,trail> matchActiveTrails
        (std::map<int,trail> &t_activeTrails,
         const int t_nodeIndex) =0;

  };












}


