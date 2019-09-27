#pragma once
#include "../MyMath.hpp"


namespace Loam{

  class DataPointInterface{
  protected:
    int m_index_container;
    bool m_isVertical;
    bool m_hasNormal;
    bool m_isClustered;
    std::vector<int> m_boundaryIndexes;

  public:

    DataPointInterface():
      m_index_container( -1),
      m_isVertical(false),
      m_hasNormal(false),
      m_isClustered(false)
    {
      m_boundaryIndexes = {-1, -1, -1 ,-1};
    };
      

    DataPointInterface( const int t_index):
      m_index_container( t_index),
      m_isVertical( false),
      m_hasNormal(false),
      m_isClustered( false)
    {
      m_boundaryIndexes = {-1, -1, -1 ,-1};
    };


    DataPointInterface(
                       const int t_index,
                       const bool t_isVert,
                       const bool t_hasNormal,
                       const bool t_isClustered
                       ):
      m_index_container( t_index),
      m_isVertical( t_isVert),
      m_hasNormal(t_hasNormal),
      m_isClustered( t_isClustered)
    {
      m_boundaryIndexes = {-1, -1, -1 ,-1};
    };


    ~DataPointInterface() = default;

    inline void setIndexContainer( const int t_index){ m_index_container = t_index;}
    inline const int& getIndexContainer() const {return  m_index_container;}
    
    inline bool getIsVertical(){ return m_isVertical;};
    inline bool getHasNormal(){ return m_hasNormal;};
    inline bool getIsClustered(){return m_isClustered;};
    inline std::vector<int> getBoundaries(){return m_boundaryIndexes;};

    inline void setIsVertical( const bool t_isVert){ m_isVertical=t_isVert;};
    inline void setHasNormal( const bool t_hasNormal){ m_hasNormal=t_hasNormal;};
    inline void setIsClustered( const bool t_isClustered){ m_isClustered=t_isClustered;};
    inline void setBoundaries( const std::vector<int> & t_bounds){ m_boundaryIndexes= t_bounds;};
    
  };

}


