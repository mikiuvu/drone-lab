/**
 * @file moving_armor_system.cc
 * @brief 通过滑动关节速度指令驱动基地装甲板沿滑轨往返。
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <sdf/Element.hh>

namespace drone_sim
{
/**
 * @brief 只控制挂载模型的一个 prismatic 关节，位置由物理引擎积分。
 * @note JointPosition 是模型内部反馈；不建立网络发布器或控制输入话题。
 *       板接触由 SDF 中的 contact 传感器和 Gazebo Contact 系统独立生成。
 */
class MovingArmorSystem final : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemPreUpdate
{
 public:
  /**
   * @brief 读取测试场景的关节名、半行程（m）与速度（m/s）。
   * @param _entity 插件所属模型；不会查找或操作无人机模型。
   * @param _sdf 模型插件参数，默认 armor_slide、0.5 m、0.2 m/s。
   */
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(_entity);
    if (_sdf->HasElement("joint_name"))
      jointName_ = _sdf->Get<std::string>("joint_name");
    if (_sdf->HasElement("half_travel_m"))
      halfTravel_ = _sdf->Get<double>("half_travel_m");
    if (_sdf->HasElement("speed_mps"))
      speed_ = _sdf->Get<double>("speed_mps");
    valid_ = model_.Valid(_ecm) && !jointName_.empty() &&
             std::isfinite(halfTravel_) && halfTravel_ > 0 &&
             std::isfinite(speed_) && speed_ >= 0;
    if (!valid_)
      gzerr << "MovingArmorSystem: invalid model or motion parameters\n";
  }

  /**
   * @brief 按真实关节位移折返，物理暂停时保持状态不变。
   * @note 最后一帧速度限为剩余距离/dt，避免一步越过端点；硬限位提供
   *       第二层约束。碰撞改变位移时以当前位移继续运动，不重置实体位姿。
   */
  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!valid_ || _info.paused)
      return;
    if (joint_ == gz::sim::kNullEntity)
    {
      joint_ = model_.JointByName(_ecm, jointName_);
      if (joint_ == gz::sim::kNullEntity)
      {
        gzerr << "MovingArmorSystem: joint not found: " << jointName_ << "\n";
        valid_ = false;
        return;
      }
      // 创建位置组件启用物理引擎的关节位置反馈，首帧等待有效测量。
      if (!_ecm.Component<gz::sim::components::JointPosition>(joint_))
        _ecm.CreateComponent(joint_, gz::sim::components::JointPosition());
    }
    if (_info.dt < std::chrono::steady_clock::duration::zero())
      direction_ = 1;
    const auto *position =
        _ecm.Component<gz::sim::components::JointPosition>(joint_);
    double velocity = 0;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (position && !position->Data().empty() && dt > 0)
    {
      const double q = position->Data()[0];
      if (std::isfinite(q))
      {
        // 端点容差随行程缩放，小行程也保留有效的往返区间。
        const double tolerance = std::min(1e-6, halfTravel_ * 1e-4);
        if (direction_ > 0 && q >= halfTravel_ - tolerance)
        {
          std::clog << "Original base carriage: endpoint y=" << q << " m\n";
          direction_ = -1;
        }
        else if (direction_ < 0 && q <= -halfTravel_ + tolerance)
        {
          std::clog << "Original base carriage: endpoint y=" << q << " m\n";
          direction_ = 1;
        }
        const double remaining = std::max(0.0, halfTravel_ - direction_ * q);
        velocity = direction_ * std::min(speed_, remaining / dt);
      }
    }
    // 速度指令交给物理引擎，保留板碰撞与关节约束；不写 Pose 或位置重置。
    _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(
        joint_, std::vector<double>{velocity});
  }

 private:
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity joint_{gz::sim::kNullEntity};
  std::string jointName_{"armor_slide"};
  double halfTravel_{0.5};
  double speed_{0.2};
  int direction_{1};
  bool valid_{false};
};
}  // namespace drone_sim

GZ_ADD_PLUGIN(drone_sim::MovingArmorSystem, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(drone_sim::MovingArmorSystem, "drone_sim::MovingArmorSystem")
