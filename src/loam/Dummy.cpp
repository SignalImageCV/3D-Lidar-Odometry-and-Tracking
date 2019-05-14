#include "Dummy.hpp"

namespace Loam {
  Dummy::Dummy(unsigned int num):
    m_num( num){};

  void Dummy::printNumCores(){
    unsigned int c = std::thread::hardware_concurrency();
    std::cout << "num of cores: "<< c << "\n";
  }

}
