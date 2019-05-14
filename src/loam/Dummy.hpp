#pragma once
#include <unistd.h>
#include <iostream>
#include <thread>


namespace Loam {
  class Dummy{
  private:
    int m_num;

  public:
    Dummy( unsigned int num );
    void printNumCores();

  };
}
