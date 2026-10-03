#include "companion/rm_auto_aim.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <cstring>

int main(int argc,char**argv){
  if(argc==2){
    cv::Mat bgr=cv::imread(argv[1]);cv::Mat rgb;cv::cvtColor(bgr,rgb,cv::COLOR_BGR2RGB);
    companion::PpmImage p;p.width=rgb.cols;p.height=rgb.rows;p.rgb.assign(rgb.data,rgb.data+rgb.total()*3);
    companion::CameraConfig c;c.fx_px=c.fy_px=320;c.cx_px=319.5;c.cy_px=179.5;c.strip_center_separation_m=.18409;
    auto r=companion::detectRmArmor(p,c,.093427,.108873);
    std::cout<<r.valid<<" "<<r.position_camera_m.transpose()<<" "<<r.reprojection_error_px<<std::endl;return r.valid?0:1;
  }

  companion::CameraConfig camera;camera.fx_px=camera.fy_px=500;camera.cx_px=319.5;camera.cy_px=239.5;
  for(double range:{.4,1.0,2.0})for(double tilt:{0.0,.25}){
    cv::Mat image(480,640,CV_8UC3,cv::Scalar(15,15,15));
    const cv::Matx33d k(500,0,319.5,0,500,239.5,0,0,1);
    for(double x:{-.0675,.0675}){
      std::vector<cv::Point3d> points{{x-.005,-.0275,0},{x+.005,-.0275,0},{x+.005,.0275,0},{x-.005,.0275,0}};
      std::vector<cv::Point2d> projected;cv::projectPoints(points,cv::Vec3d(.08,tilt,.05),cv::Vec3d(.02,.01,range),k,cv::noArray(),projected);
      std::vector<cv::Point> polygon;for(auto p:projected)polygon.emplace_back(cvRound(p.x),cvRound(p.y));
      cv::fillConvexPoly(image,polygon,cv::Scalar(0,15,255)); // RGB: blue
    }
    companion::PpmImage ppm;ppm.width=640;ppm.height=480;ppm.rgb.assign(image.data,image.data+image.total()*3);
    auto result=companion::detectRmArmor(ppm,camera);
    assert(result.valid);assert(std::abs(result.position_camera_m.z()-range)<range*.15);
    assert(result.reprojection_error_px<3);
    for(size_t i=0;i<ppm.rgb.size();i+=3)std::swap(ppm.rgb[i],ppm.rgb[i+2]);
    assert(!companion::detectRmArmor(ppm,camera).valid);
  }
  // 原基地正面梯形灯条，上端间距大于下端；端点模型必须与观测一致。
  {
    camera.strip_center_separation_m=.18409;
    cv::Mat image(480,640,CV_8UC3,cv::Scalar(15,15,15));
    const cv::Matx33d k(500,0,319.5,0,500,239.5,0,0,1);
    for(double side:{-1.0,1.0}){
      std::vector<cv::Point3d> points{{side*.092-.02,-.0467,0},{side*.092+.02,-.0467,0},{side*.0544+.02,.0467,0},{side*.0544-.02,.0467,0}};
      std::vector<cv::Point2d> projected;cv::projectPoints(points,cv::Vec3d(0,.12,.05),cv::Vec3d(0,0,.8),k,cv::noArray(),projected);
      std::vector<cv::Point> poly;for(auto p:projected)poly.emplace_back(cvRound(p.x),cvRound(p.y));cv::fillConvexPoly(image,poly,cv::Scalar(0,15,255));
    }
    companion::PpmImage ppm;ppm.width=640;ppm.height=480;ppm.rgb.assign(image.data,image.data+image.total()*3);
    auto result=companion::detectRmArmor(ppm,camera,.093427,.108873);
    assert(result.valid&&std::abs(result.position_camera_m.z()-.8)<.08);
  }
  companion::PpmImage blank;blank.width=640;blank.height=480;blank.rgb.resize(640*480*3);
  assert(!companion::detectRmArmor(blank,camera).valid);
}
