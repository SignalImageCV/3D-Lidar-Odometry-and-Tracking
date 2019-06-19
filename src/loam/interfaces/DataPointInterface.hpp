#pragma once
#include "../MyMath.hpp"

namespace Loam{

  class DataPointInterface{
    protected:
    int m_index_container;
    bool m_isVertical;
    bool m_isTaken;

    public:

      DataPointInterface():
        m_index_container( -1),
        m_isVertical(false),
        m_isTaken(false)
      {};
      

      DataPointInterface( const int t_index):
        m_index_container( t_index),
        m_isVertical( false),
        m_isTaken( false)
      {};


      DataPointInterface(
          const int t_index,
          const bool t_isVert,
          const bool t_isTaken
          ):
        m_index_container( t_index),
        m_isVertical( t_isVert),
        m_isTaken( t_isTaken)
      {};


      ~DataPointInterface() = default;

      inline int getIndexContainer(){return  m_index_container;};
      inline bool getIsVertical(){ return m_isVertical;};
      inline bool getIsTaken(){return m_isTaken;};
      inline void setIndexContainer( const int t_index){ m_index_container = t_index;};
      inline void setIsVertical( const bool t_isVert){ m_isVertical=t_isVert;};
      inline void setIsTaken( const bool t_isTaken){ m_isVertical=t_isTaken;};

  };

}


