#include "companion/terrain.hpp"
#include "companion/estimator.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>

// Prior-map design analysis only; never a source of live vehicle observations.
int main(int argc,char**argv){
  if(argc!=2)return 2;
  auto map=companion::TerrainMap::load(argv[1]);
  struct Candidate{double x,y,z,yaw,score;};std::vector<Candidate> candidates;
  for(double x: {-3.5,-2.5,2.5,3.5})for(double y: {-4.0,-3.0,3.0,4.0}){
    companion::TerrainMap::Hit ground;
    if(!map->raycast({x,y,5},{0,0,-1},6,ground)||ground.point.z()>1.3)continue;
    double floor=ground.point.z();
    for(double yaw=0;yaw<6.28;yaw+=1.0471975512){
      double score=1e10;
      for(double z: {floor+.0125,1.0,1.8}){
        if(z<floor+.01){score=0;break;}
        Eigen::Matrix3d information=Eigen::Matrix3d::Zero();int hits=0;
        for(int s=0;s<3;++s){
          double angle=yaw+s*2*companion::kPi/3;
          Eigen::Matrix3d r=Eigen::AngleAxisd(angle,Eigen::Vector3d::UnitZ()).toRotationMatrix();
          Eigen::Vector3d off=r*Eigen::Vector3d(.055,0,0),origin=Eigen::Vector3d(x,y,z)+off;
          for(int row=0;row<8;++row)for(int col=0;col<8;++col){
            Eigen::Vector3d ray=r*Eigen::Vector3d(1,-(col-3.5)/4*std::tan(companion::kPi/8),-(row-3.5)/4*std::tan(companion::kPi/8));
            companion::TerrainMap::Hit h;
            if(!map->raycast(origin,ray,4,h)||h.depth_m<.1)continue;
            Eigen::Vector3d lever=off+h.depth_m*ray;
            Eigen::Vector3d j(h.normal.x(),h.normal.y(),h.normal.dot(Eigen::Vector3d(-lever.y(),lever.x(),0)));
            information+=j*j.transpose();++hits;
          }
        }
        Eigen::Vector3d values=Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(information).eigenvalues();
        score=std::min(score,values(0));
      }
      if(score>.1)candidates.push_back({x,y,floor+.2,yaw,score});
    }
  }
  std::sort(candidates.begin(),candidates.end(),[](auto a,auto b){return a.score>b.score;});
  for(size_t i=0;i<std::min(size_t(80),candidates.size());++i){auto c=candidates[i];std::cout<<c.x<<","<<c.y<<","<<c.z<<","<<c.yaw<<","<<c.score<<std::endl;}
}
