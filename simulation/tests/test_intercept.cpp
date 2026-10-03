#include "companion/intercept.hpp"
#include <cassert>
#include <limits>
int main(){
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
