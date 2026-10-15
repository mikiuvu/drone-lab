/*
 * Adapted from chenjunnn/rm_auto_aim, commit
 * f244d0e296327d73171311ec50bf5f88856077b1:
 * armor_detector/src/detector.cpp and armor_detector/src/pnp_solver.cpp.
 * Uses minAreaRect light endpoints, geometric pairing and four-point IPPE;
 * RGB blue screening and multi-solution reprojection selection are adapted
 * to the companion's image and camera interface.
 *
 * MIT License
 *
 * Copyright (c) 2022 ChenJun
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "companion/rm_auto_aim.hpp"
#include "companion/color_threshold.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace companion {
namespace {

struct Light {
  cv::Point2f center;
  cv::Point2f top;
  cv::Point2f bottom;
  double length;
  double width;
};

bool validPnpInput(const PpmImage& image,const CameraConfig& camera,double strip_height_m){
  return image.width>0&&image.height>0&&
      image.rgb.size()==static_cast<size_t>(image.width)*static_cast<size_t>(image.height)*3&&
      std::isfinite(camera.fx_px)&&std::isfinite(camera.fy_px)&&
      std::isfinite(camera.cx_px)&&std::isfinite(camera.cy_px)&&
      std::isfinite(camera.strip_center_separation_m)&&std::isfinite(strip_height_m)&&
      camera.fx_px>0&&camera.fy_px>0&&camera.strip_center_separation_m>0&&strip_height_m>0;
}

std::vector<Light> findLights(const PpmImage& image,bool trapezoid,std::vector<uint8_t> pixels) {
  cv::Mat mask(image.height,image.width,CV_8UC1,pixels.data());
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  std::vector<Light> lights;
  for (const auto& contour : contours) {
    if (contour.size() < 4 || cv::contourArea(contour) < 3.0) continue;
    const cv::Rect bounds = cv::boundingRect(contour);
    if (bounds.x <= 0 || bounds.y <= 0 || bounds.br().x >= image.width ||
        bounds.br().y >= image.height) continue;
    const auto rect = cv::minAreaRect(contour);
    cv::Point2f vertices[4];
    rect.points(vertices);
    const double edge0 = cv::norm(vertices[1] - vertices[0]);
    const double edge1 = cv::norm(vertices[2] - vertices[1]);
    // 短边中点给出中心线端点，避免矩形对角线引入灯条宽度偏差。
    cv::Point2f a, b;
    if (edge0 >= edge1) {
      a = (vertices[0] + vertices[3]) * 0.5F;
      b = (vertices[1] + vertices[2]) * 0.5F;
    } else {
      a = (vertices[0] + vertices[1]) * 0.5F;
      b = (vertices[2] + vertices[3]) * 0.5F;
    }
    if (a.y > b.y) std::swap(a, b);
    if(trapezoid){
      std::vector<cv::Point> polygon;
      cv::approxPolyDP(contour,polygon,.025*cv::arcLength(contour,true),true);
      // 四顶点梯形使用上下边中点；其他轮廓保留最小外接矩形的中心线端点。
      if(polygon.size()==4){
        std::sort(polygon.begin(),polygon.end(),[](auto p,auto q){return p.y<q.y;});
        a=(cv::Point2f(polygon[0])+cv::Point2f(polygon[1]))*.5F;
        b=(cv::Point2f(polygon[2])+cv::Point2f(polygon[3]))*.5F;
      }
    }
    const double length = cv::norm(b - a);
    const double width = std::min(edge0, edge1);
    const double tilt = std::atan2(std::abs(b.x - a.x), b.y - a.y);
    if (length < 5.0 || width < 0.5 || width / length < 0.02 ||
        width / length > 0.6 || tilt > 35.0 * CV_PI / 180.0) continue;
    lights.push_back({rect.center, a, b, length, width});
  }
  std::sort(lights.begin(), lights.end(), [](const Light& a, const Light& b) {
    return a.center.x < b.center.x;
  });
  return lights;
}

double pairScore(const std::vector<Light>& lights, size_t i, size_t j, bool trapezoid,
                 double stripHeightM) {
  const auto& left = lights[i];
  const auto& right = lights[j];
  const cv::Point2f delta = right.center - left.center;
  const double ratio = std::min(left.length, right.length) / std::max(left.length, right.length);
  const double average = (left.length + right.length) * 0.5;
  const double gap_ratio = cv::norm(delta) / average;
  const double angle = std::atan2(std::abs(delta.y), delta.x);
  const double parallel = (left.bottom - left.top).dot(right.bottom - right.top) /
                          (left.length * right.length);
  // 小装甲灯条宽约 5 mm。斜视使长边投影缩短，近距离透视也使两条灯条
  // 向消失点收敛；只在宽高比显示明显缩短时允许较大的夹角，PnP 继续筛选。
  const double lampAspect = 0.005 / stripHeightM;
  const double observedAspect =
      0.5 * (left.width / left.length + right.width / right.length);
  const bool stronglyForeshortened = !trapezoid && observedAspect > lampAspect * 1.2;
  const double maxParallelAngle = stronglyForeshortened ? 45.0 :
      (trapezoid ? 60.0 : 20.0);
  if (delta.x < 3.0 || ratio < 0.6 || gap_ratio < 1.0 || gap_ratio > 5.0 ||
      angle > 35.0 * CV_PI / 180.0 ||
      parallel < std::cos(maxParallelAngle * CV_PI / 180.0)) return 0.0;
  // 中间存在另一灯条时不跨条配对。
  const cv::Rect box = cv::boundingRect(std::vector<cv::Point2f>{
      left.top, left.bottom, right.top, right.bottom});
  for (size_t k = i + 1; k < j; ++k) {
    if (box.contains(lights[k].center)) return 0.0;
  }
  return average * ratio * std::cos(angle);
}

RmArmorDetection solvePair(const Light& left, const Light& right,
                          const CameraConfig& camera, double height, double bottom) {
  RmArmorDetection result;
  const std::vector<cv::Point2d> pixels{left.bottom, left.top, right.top, right.bottom};
  const double w = camera.strip_center_separation_m * 0.5, h = height * 0.5;
  // 局部板坐标 x 向右、y 向下；原点在板中心，z=0 为板平面。
  const double lower=bottom>0?bottom*.5:w;
  const std::vector<cv::Point3d> model{{-lower, h, 0}, {-w, -h, 0}, {w, -h, 0}, {lower, h, 0}};
  const cv::Matx33d intrinsics(camera.fx_px, 0, camera.cx_px, 0, camera.fy_px,
                             camera.cy_px, 0, 0, 1);
  std::vector<cv::Mat> rotations, translations;
  cv::solvePnPGeneric(model, pixels, intrinsics, cv::noArray(), rotations, translations,
                      false, cv::SOLVEPNP_IPPE);
  for (size_t i = 0; i < translations.size(); ++i) {
    if (!cv::checkRange(translations[i]) || !cv::checkRange(rotations[i])) continue;
    const cv::Vec3d t = translations[i];
    cv::Matx33d rotation;
    cv::Rodrigues(rotations[i], rotation);
    bool in_front = t[2] > 0;
    for (const auto& point : model) {
      const cv::Vec3d transformed = rotation * cv::Vec3d(point.x, point.y, point.z) + t;
      in_front = in_front && transformed[2] > 0;
    }
    if (!in_front) continue;
    std::vector<cv::Point2d> projected;
    cv::projectPoints(model, rotations[i], translations[i], intrinsics, cv::noArray(), projected);
    double squared_error = 0.0;
    for (size_t k = 0; k < pixels.size(); ++k) {
      const auto difference = projected[k] - pixels[k];
      squared_error += difference.dot(difference);
    }
    const double error = std::sqrt(squared_error / pixels.size());
    if (!std::isfinite(error) || error >= result.reprojection_error_px) continue;
    result.reprojection_error_px = error;
    result.position_camera_m = Eigen::Vector3d(t[0], t[1], t[2]);
    for (size_t k = 0; k < pixels.size(); ++k) {
      result.corners[k] = Eigen::Vector2d(pixels[k].x, pixels[k].y);
    }
  }
  result.valid = result.position_camera_m.allFinite() && result.position_camera_m.z() > 0 &&
                 result.reprojection_error_px <= 3.0;
  return result;
}

}  // namespace

RmArmorDetection detectRmArmor(const PpmImage& image, const CameraConfig& camera,
                               double strip_height_m, double bottom_separation_m) {
  if(!validPnpInput(image,camera,strip_height_m))return {};
  std::vector<uint8_t> mask(static_cast<size_t>(image.width)*image.height);
  for(size_t i=0;i<mask.size();++i){
    const auto* rgb=image.rgb.data()+i*3;
    mask[i]=detail::pnpBluePixel(rgb[0],rgb[1],rgb[2])?255:0;
  }
  return detectRmArmor(image,camera,strip_height_m,bottom_separation_m,std::move(mask));
}

RmArmorDetection detectRmArmor(const PpmImage& image,const CameraConfig& camera,
    double strip_height_m,double bottom_separation_m,std::vector<uint8_t> mask){
  RmArmorDetection result;
  if(!validPnpInput(image,camera,strip_height_m)||mask.size()!=static_cast<size_t>(image.width)*image.height)return result;
  const auto lights = findLights(image,bottom_separation_m>0,std::move(mask));
  double best_score = 0;
  for (size_t i = 0; i < lights.size(); ++i) {
    for (size_t j = i + 1; j < lights.size(); ++j) {
      const double geometry_score = pairScore(
          lights, i, j, bottom_separation_m > 0, strip_height_m);
      if (geometry_score <= 0) continue;
      const auto candidate = solvePair(lights[i], lights[j], camera, strip_height_m, bottom_separation_m);
      const double score = geometry_score / (1.0 + candidate.reprojection_error_px);
      if (candidate.valid && score > best_score) {
        best_score = score;
        result = candidate;
      }
    }
  }
  return result;
}

}  // namespace companion
