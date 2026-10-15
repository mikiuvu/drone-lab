#include "companion/vision_worker.hpp"
#include "companion/latest_tof.hpp"
#include <cassert>
#include <chrono>
#include <thread>

using namespace companion;
namespace {
VisionRequest request(uint64_t stamp) {
  VisionRequest r; r.stamp = stamp; r.sequence = stamp;
  PpmImage image;image.width=640;image.height=360;image.rgb.assign(640*360*3,15);
  for(int left:{282,354})for(int y=162;y<=198;++y)for(int x=left;x<=left+4;++x){
    const auto pixel=(y*640+x)*3;image.rgb[pixel]=0;image.rgb[pixel+1]=15;image.rgb[pixel+2]=255;
  }
  r.image = std::make_shared<const PpmImage>(std::move(image));
  r.verify_metadata = false;
  r.camera.width = r.image->width; r.camera.height = r.image->height;
  r.camera.fx_px = r.camera.fy_px = r.image->width * .5;
  r.camera.cx_px = (r.image->width - 1) * .5; r.camera.cy_px = (r.image->height - 1) * .5;
  r.camera.strip_center_separation_m = .116;
  r.detection.enemy_color = ArmorColor::Blue; r.detection.max_pair_aspect = 6;
  r.strip_height_m = .055; r.bottom_separation_m = 0;
  r.pose_valid = true; r.exposure_pose = {10.6, 0, 1.44, 0, 0, 0};
  r.exposure_covariance.diagonal()<<.0004,.0025,.0009,.0001;
  r.exposure_covariance(1,3)=r.exposure_covariance(3,1)=.0002;
#ifndef COMPANION_WITH_OPENCV
  r.pnp = false;
#endif
  return r;
}
std::optional<VisionResult> wait(VisionWorker& w, uint64_t stamp) {
  const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < end) {
    auto result = w.poll(stamp, 10000000);
    if (result && result->stamp == stamp) return result;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  assert(false); return std::nullopt;
}
}
int main() {
#ifdef COMPANION_WITH_OPENCV
  for(const auto color:{std::pair<int,int>{180,230},{20,140}}){
    auto input=request(1000000);
    auto image=std::make_shared<PpmImage>(*input.image);
    for(size_t i=0;i<image->rgb.size();i+=3)if(image->rgb[i+2]==255){image->rgb[i+1]=color.first;image->rgb[i+2]=color.second;}
    input.image=image;
    VisionWorker worker(false);
    assert(worker.submit(input));const auto both=wait(worker,input.stamp);
    assert(!both->pixel.valid&&both->pnp.valid&&both->world_valid);
    input.stamp+=20000;input.pixel=false;input.detection.blue_over_green_ratio=0;
    assert(worker.submit(input));const auto pnp=wait(worker,input.stamp);
    assert(pnp->error.empty()&&pnp->world_valid&&(pnp->world-both->world).norm()<1e-12);
    assert(!detectRmArmor(*image,input.camera,input.strip_height_m,input.bottom_separation_m,std::vector<uint8_t>(1)).valid);
    input.stamp+=20000;input.pixel=true;input.pnp=false;input.detection.blue_over_green_ratio=1.4;
    assert(worker.submit(input));const auto pixel=wait(worker,input.stamp);
    assert(pixel->error.empty()&&!pixel->pixel.valid&&!pixel->pnp.valid&&!pixel->world_valid);
  }
#endif
  const auto input = request(1000000);
  const auto reference = detectArmorStrips(*input.image, input.camera, input.detection);
  VisionWorker synchronous(false), background;
  assert(synchronous.submit(input)); assert(background.submit(input));
  const auto a = wait(synchronous, input.stamp), b = wait(background, input.stamp);
  assert(a->frame_valid && b->frame_valid && a->pixel.valid == reference.valid);
  assert(a->pixel.range_m == b->pixel.range_m && b->pixel.range_m == reference.range_m);
  assert(a->pixel.centroid_x_px == b->pixel.centroid_x_px);
  assert((a->world - b->world).norm() < 1e-12);
  assert((a->world_covariance-b->world_covariance).norm()<1e-12);
#ifdef COMPANION_WITH_OPENCV
  const auto detected=detectRmArmor(*input.image,input.camera,input.strip_height_m,input.bottom_separation_m);
  const Eigen::Vector3d expected(input.exposure_pose.x_m+detected.position_camera_m.z()+input.camera_forward_m,
      input.exposure_pose.y_m-detected.position_camera_m.x(),input.exposure_pose.z_m-detected.position_camera_m.y());
  assert(detected.valid&&a->world_valid&&(a->world-expected).norm()<1e-12);
  assert(a->world_covariance(1,1)>input.pnp_variance_m2+.0025);
#endif
  assert(!synchronous.submit(input));

  auto mismatch = input; mismatch.stamp += 1; mismatch.verify_metadata = true;
  assert(synchronous.submit(mismatch));
  auto invalid = wait(synchronous, mismatch.stamp);
  assert(!invalid->frame_valid && invalid->error == "camera_frame_mismatch");
  auto next = input; next.stamp += 2;
  assert(synchronous.submit(next)); assert(!synchronous.poll(next.stamp - 1, 100000));
  next.stamp += 1; assert(synchronous.submit(next)); assert(!synchronous.poll(next.stamp + 101, 100));
  assert(synchronous.stats().future == 1 && synchronous.stats().stale == 1);

  auto green_request=input;green_request.stamp=2000000;
  auto green_image=std::make_shared<PpmImage>(*input.image);green_image->rgb.assign(640*360*3,0);
  for(int y=178;y<=182;++y)for(int x=318;x<=322;++x)green_image->rgb[(y*640+x)*3+1]=255;
  green_request.image=green_image;green_request.guide_mode=GuideMode::Fallback;
  green_request.guide_x_px=320;green_request.guide_y_px=180;green_request.guide_radius_px=30;
  green_request.guide_center={11.58,0,1.44};green_request.guide_y_gate_m=.4;green_request.guide_z_gate_m=.1;
  assert(synchronous.submit(green_request));auto green=wait(synchronous,green_request.stamp);
  assert(green->green_ran&&green->green&&green->world_valid&&!green->pnp.valid);
  assert(std::abs(green->world.x()-11.58)<1e-12&&std::abs(green->world.y())<.01);
  assert(std::abs(green->world_covariance(0,0)-green_request.guide_variance_m2)<1e-12);
  assert(green->world_covariance(1,1)>green_request.guide_variance_m2+.0025);

#ifdef COMPANION_WITH_OPENCV
  next.stamp=green_request.stamp+1;next.guide_mode = GuideMode::Fallback;
  assert(synchronous.submit(next)); auto fallback = wait(synchronous, next.stamp);
  assert(fallback->pnp.valid && fallback->world_valid && !fallback->green_ran);
#endif
  auto invalid_covariance=green_request;invalid_covariance.stamp+=100;
  invalid_covariance.exposure_covariance(0,0)=-1;
  assert(synchronous.submit(invalid_covariance));auto bad_covariance=wait(synchronous,invalid_covariance.stamp);
  assert(bad_covariance->frame_valid&&!bad_covariance->world_valid&&bad_covariance->error=="invalid_guide_world_uncertainty");
  // 连续输入不累积图像队列，最终仍可取得最新帧。
  uint64_t latest = input.stamp;
  for (int i = 1; i <= 100; ++i) {
    auto r = input; r.stamp = latest + 20000;
    while (!background.submit(r)) std::this_thread::yield();
    latest = r.stamp;
  }
  assert(wait(background, latest)->frame_valid);
  assert(background.stats().pending_replaced > 0);
  auto copied=green_request;copied.stamp=latest+20000;
  assert(background.submit(copied));copied.exposure_covariance.setZero();
  const auto owned=wait(background,copied.stamp);
  assert(owned->world_valid&&owned->green&&owned->world_covariance(1,1)>copied.guide_variance_m2+.0025);

  LatestTof queue;
  TofFrame frame; frame.sensor_index = 0; frame.sim_time_us = 10;
  assert(queue.submit(frame)); assert(!queue.submit(frame));
  frame.sim_time_us = 30; assert(queue.submit(frame));
  frame.sensor_index = 1; frame.sim_time_us = 20; assert(queue.submit(frame));
  frame.sensor_index = 2; frame.sim_time_us = 40; assert(queue.submit(frame));
  assert(queue.take()->sim_time_us == 20);
  assert(queue.take()->sim_time_us == 30);
  assert(queue.take()->sim_time_us == 40);
  assert(!queue.take() && queue.replaced == 1 && queue.taken == 3);
}
