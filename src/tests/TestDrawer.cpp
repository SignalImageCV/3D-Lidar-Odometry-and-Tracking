#include <gtest/gtest.h>
#include "../loam/Drawer.hpp"

using namespace testing;
using namespace Loam;

int main(int argc, char** argv) {
    RGBImage dImg;
    dImg.create( 900, 1200);
    dImg= cv::Vec3b(255, 255, 255);
    cv::namedWindow("Poligon");
    cv::Scalar dark_red = {20,0,255};

    cv::Point2d p1 ( 10 , 10);
    cv::Point2d p2 ( 120 , 10);
    cv::Point2d p3 ( 20 , 120);
    std::vector<cv::Point2d> points = { p1, p2, p3};
    int scale = 10;
    Drawer d = Drawer(scale);
    d.drawHollowPoligon(dImg, points, dark_red);
    cv::imshow("Poligon",dImg);
    cv::waitKey();
  
    return 0;
}



