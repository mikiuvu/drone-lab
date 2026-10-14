#include "companion/intercept.hpp"
#include "companion/rail_motion.hpp"
#include <cassert>
#include <limits>
int main(){
 companion::RailMotionFit fit(.3);
 for(int i=0;i<12;++i){
   const double position=.2*(i*.066)+(i==7?.25:0);
   assert(fit.observe(1000000+i*66000,position));
 }
 auto smooth=fit.at(1760000);
 assert(smooth&&std::abs(smooth->velocity_mps-.2)<1e-10&&std::abs(smooth->position_m-.152)<1e-10);
 assert(!fit.observe(1700000,3));
 assert(!fit.at(2000000));
 assert(fit.observe(3000000,.1)&&!fit.at(3000000));
 auto bounded=companion::predictRailLateral(.1,1,.5,0,.3);
 assert(bounded&&std::abs(*bounded-.25)<1e-12);
 auto reversal=companion::predictRailLateral(.27,.2,.2,0,.3,.28);
 assert(reversal&&std::abs(*reversal-.25)<1e-12);
 auto reverse_left=companion::predictRailLateral(-.27,-.2,.2,0,.3,.28);
 assert(reverse_left&&std::abs(*reverse_left+.25)<1e-12);
 auto long_motion=companion::predictRailLateral(.1,.2,8,0,.3,.28);
 assert(long_motion&&*long_motion>=-.28&&*long_motion<=.28);
 auto stationary_edge=companion::predictRailLateral(.5,0,.5,0,.3,.28);
 assert(stationary_edge&&std::abs(*stationary_edge-.28)<1e-12);
 assert(!companion::predictRailLateral(0,0,-1,0,.3));
 assert(!companion::predictRailLateral(0,0,1,0,0));
 assert(!companion::predictRailLateral(0,std::numeric_limits<double>::infinity(),1,0,.3));
 auto ramp=companion::estimateArrivalTime(.75,2.0,3.0,1.5);
 assert(ramp&&std::abs(*ramp-1.0/3.0)<1e-12);
 auto capped=companion::estimateArrivalTime(3,2,3,1.5);
 assert(capped&&std::abs(*capped-10.0/9.0)<1e-12);
 auto backed=companion::estimateArrivalTime(.75,-2,3,1.5);
 assert(backed&&*backed>*ramp);
 assert(!companion::estimateArrivalTime(1,0,3,0));
 assert(!companion::estimateArrivalTime(-1,0,3,1));
 assert(!companion::estimateArrivalTime(1,std::numeric_limits<double>::quiet_NaN(),3,1));
 assert(!companion::estimateArrivalTime(1,0,3,std::numeric_limits<double>::denorm_min()));
 assert(!companion::estimateArrivalTime(1,1e200,2e200,1e200));
 double aim=.2;uint64_t exposure=1000000;
 assert(companion::correctLateralAim(aim,exposure,.02,1060000,1120000,1.0));
 assert(aim<.2&&aim>=.14-1e-12);
 const double corrected=aim;
 assert(!companion::correctLateralAim(aim,exposure,-.2,1060000,1140000,1.0)&&aim==corrected);
 assert(!companion::correctLateralAim(aim,exposure,-.2,1070000,1240000,1.0)&&aim==corrected);
 assert(!companion::correctLateralAim(aim,exposure,-.2,1500000,1400000,1.0)&&aim==corrected);
 assert(companion::correctLateralAim(aim,exposure,.02,1200000,1240000,1.0));
 assert(std::abs(aim-.02)<1e-12);
 using Eigen::Vector3d;using companion::planIntercept;
 auto moving=planIntercept(Vector3d::Zero(),Vector3d::Zero(),{.4,0,0},{0,.2,0});
 assert(moving.valid&&moving.aim.y()>0&&moving.velocity.y()>0&&moving.arrival_s<=.8);
 auto forward=planIntercept(Vector3d::Zero(),{.6,0,0},{.4,0,0},Vector3d::Zero());
 auto reverse=planIntercept(Vector3d::Zero(),{-.6,0,0},{.4,0,0},Vector3d::Zero());
 assert(forward.arrival_s<reverse.arrival_s);
 auto far=planIntercept(Vector3d::Zero(),Vector3d::Zero(),{10,0,0},{0,.2,0});
 assert(!far.reachable&&far.aim.y()<=.161&&far.velocity.norm()<=1.20001);
 auto flipped=planIntercept(Vector3d::Zero(),Vector3d::Zero(),{.4,0,0},{0,-.2,0});
 assert(flipped.aim.y()<0);
 assert(!planIntercept(Vector3d::Zero(),Vector3d::Zero(),{1,0,0},{0,0,0},0).valid);
}
