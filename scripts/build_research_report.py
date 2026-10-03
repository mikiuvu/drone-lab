"""@file build_research_report.py
@brief 读取已保存研究结果和风险清单，生成 docs/research/index.html 总览。
@note 写入会覆盖现有总览；只使用与源码摘要匹配的结果。
"""

import html
import json
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
e = html.escape


def verify_sources(bundle, root=ROOT):
    """@brief 检查每个结果来源位于项目内且 SHA256 与当前完整文件一致。
    @param bundle 保存的 runs 字典所在结果包。
    @param root 项目根路径。
    @note 失败抛 ValueError；源码注释变化也会使已有摘要过期。
    """
    for name, run in bundle["runs"].items():
        source = (root / run["source"]).resolve()
        if not source.is_relative_to(root.resolve()):
            raise ValueError("Source outside project")
        if hashlib.sha256(source.read_bytes()).hexdigest() != run["source_sha256"]:
            raise ValueError(
                f"{name}: source changed; rerun scripts/run_research.py before publishing"
            )


def numeric(value):
    """@brief 将空统计值显示为无完成样本，数值保留一位小数。
    """
    return "无完成样本" if value is None else f"{value:.1f}"


def main():
    """@brief 校验来源并从现有指标生成 HTML，覆盖 index.html；不重新计算模型。
    """
    bundle = json.loads(
        (ROOT / "research" / "results" / "baseline.json").read_text(encoding="utf-8")
    )
    verify_sources(bundle)
    risk = json.loads(
        (ROOT / "research" / "risk_register.json").read_text(encoding="utf-8")
    )
    compute = bundle["runs"]["compute_overload"]["result"]["metrics"]
    fifo, latest = compute["fifo"], compute["latest"]
    cfg = bundle["runs"]["compute_overload"]["result"]["config"]
    geo = bundle["runs"]["geometry_horizontal"]["result"]
    down = bundle["runs"]["geometry_downward"]["result"]
    integration = bundle["runs"]["integration_50ms"]["result"]
    bandwidth = bundle["runs"]["bandwidth_400khz"]["result"]
    control = bundle["runs"]["control_faults"]["result"]
    rows = "".join(
        "<tr><td>"
        + e(x["id"])
        + "</td><td>"
        + e(x["problem"])
        + "</td><td>"
        + e(risk["status_values"][x["status"]])
        + "</td><td>"
        + e(x["action"])
        + "</td><td>"
        + e(x["evidence_needed"])
        + "</td></tr>"
        for x in risk["risks"]
    )
    page = f"""<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><title>RM2027 无人机预研总览</title>
<style>body{{font:16px 'Microsoft YaHei UI',sans-serif;line-height:1.7;margin:30px;color:#19334b;max-width:1180px}}h1,h2{{color:#126884}}table{{border-collapse:collapse;width:100%;margin:16px 0}}td,th{{border:1px solid #ccdce8;padding:10px;text-align:left;vertical-align:top}}img{{max-width:100%}}code,pre{{background:#eef3f8}}.note{{padding:16px;background:#fff3d4}}.cards{{display:flex;gap:14px;flex-wrap:wrap}}.card{{padding:16px;background:#edf5fa;border-radius:8px}}a{{color:#08758f}}</style></head><body>
<h1>RV Nano 同时做视觉和定位：现在能确定什么</h1>
<p>日期：2026-10-02。对象：DM-FC01 + LicheeRV Nano / SG2002 + 两颗 VL53L5CX，后续增加相机与可选 UWB。研究采用电脑仿真与实现分析；硬件性能待板上测量。</p>
<div class="note"><b>结论：</b>Nano 是否够用，不能由 1 TOPS 或电脑运行速度判定。当前更值得推进的基线是单 Linux CPU 预算、低频检测与必要的 ROI 跟踪、NPU独立推理、固定频率的小规模定位任务、有界最新帧槽，以及独立的状态新鲜度检查。最大风险同时来自视觉链路适配、应用可用内存、观测覆盖和时序，不只是数学计算量。</div>
<div class="cards"><div class="card"><a href="compute.html">1. 算力、内存、相机与 NPU</a></div><div class="card"><a href="geometry.html">2. 双 ToF 几何仿真</a></div><div class="card"><a href="implementation.html">3. 最终代码与飞控接口</a></div><div class="card"><a href="collector-audit.html">4. 采集器接口与测试</a></div><div class="card"><a href="control-readiness.html">5. 失效与恢复状态机</a></div><div class="card"><a href="image-audit.html">6. Nano 镜像内容</a></div></div>
<h2>三个需要先调整的方案假设</h2>
<ol><li><b>当前没有已证实的连续里程计。</b>飞控 IMU 和两颗水平 ToF 不自动构成全场可靠 x/y/yaw。原方案的光流+下视测距被暂时移除后，需要补回这个角色，或者证明结构观测与有界预测在所有任务航段足够。</li>
<li><b>标称 256 MB 不是全部可用堆内存。</b>官方在线配置存在较大的 ION/多媒体保留区。具体镜像的内存分区、VB池和模型工作区必须在后续板端测量；主核与辅助核也不能默认等同两个可用 Linux CPU 核。</li>
<li><b>实时性看结果年龄和尾延迟。</b>低 CPU 百分比也可能伴随严重视觉积压。为避免旧目标继续影响导航，帧序号、源时间戳、epoch、队列上限和过期丢弃是代码接口的一部分。</li></ol>
<h2>电脑仿真结果</h2>
<p>以下布局、计算耗时和噪声都是显式合成假设，可在 Drone Lab 的“⑥ 预研与仿真”页面修改并重跑；完整参数见 <a href="../../research/results/baseline.json">baseline.json</a>。</p>
<table><tr><th>测试</th><th>输入与结果</th><th>适用条件</th></tr>
<tr><td>视觉过载排队</td><td>{cfg["camera_fps"]:g} fps 输入，CPU预处理{cfg["cpu_pre_ms"]:g} ms、后处理{cfg["cpu_post_ms"]:g} ms、NPU{cfg["npu_ms"]:g} ms、导航{cfg["nav_hz"]:g} Hz×{cfg["nav_ms"]:g} ms，运行{cfg["duration_s"]:g}秒。FIFO期末待完成 {fifo["frames"]["outstanding"]} 帧；已完成帧龄p95 {numeric(fifo["frames"]["age_ms"]["p95"])} ms。最新帧策略丢弃 {latest["frames"]["dropped"]} 帧、峰值持有 {latest["memory"]["max_held_frames"]} 帧、已完成帧龄p95 {numeric(latest["frames"]["age_ms"]["p95"])} ms，仍有 {latest["frames"]["stale_completed"]} 个已完成结果超过{cfg["frame_deadline_ms"]:g} ms。</td><td>有界队列仅限制应用帧引用，不含SDK/VB池；不能消除服务时间或确保每帧及时。所有服务时间与期限为研发输入，不是板上测试。</td></tr>
<tr><td>局部可观测性与覆盖</td><td>{e(geo["summary"])}</td><td>局部秩的计算条件为已知关联及已知高度/roll/pitch；连续定位需评估全局匹配与观测覆盖。</td></tr>
<tr><td>改变安装俯仰</td><td>{e(down["summary"])}</td><td>要优化视场和航路，而非只加滤波。真实区域脚印、反光、材料和遮挡会进一步改变结果。</td></tr>
<tr><td>时间错配</td><td>{e(integration["summary"])}</td><td>解析误差敏感性，强调对时和运动补偿；不把两种误差简单求和或当实测RMSE。</td></tr>
<tr><td>总线预算</td><td>{e(bandwidth["summary"])} UART RX {bandwidth["metrics"]["fc_to_nano_rx_utilization"] * 100:.1f}%、TX {bandwidth["metrics"]["nano_to_fc_tx_utilization"] * 100:.1f}%，按全双工分别计算。</td><td>默认1500字节/帧、2颗15 Hz、400 kHz。载荷须用实际驱动字节数替换；还未计时钟拉伸和操作系统开销。</td></tr>
<tr><td>失效与恢复</td><td>{e(control["summary"])}</td><td>离线状态机验证数据准入和恢复逻辑，输出决策和恢复状态。</td></tr></table>
<p><img src="figures/compute_overload.png" alt="过载时两种队列策略对比"></p>
<p><img src="figures/geometry_horizontal.png" alt="水平安装的合成几何与可观测性"></p>
<p><img src="figures/geometry_downward.png" alt="下俯安装的合成几何"></p>
<h2>如何在图形界面复现</h2>
<ol><li>重新打开桌面“Drone Lab 实验台”，选择“⑥ 预研与仿真”。</li><li>选择“任务调度与图像排队”，先默认运行，再将 NPU 服务时间改为60 ms、CPU预处理6 ms、后处理4 ms，重跑比较。</li><li>选择“双 ToF 几何与可观测性”，先保持高度2.5 m、量程3 m、安装俯仰0°，再改为−30°。正角朝上，负角朝下。</li><li>选择“时间错配与接口门控”，把错配从50 ms改成10 ms，查看误差变化。</li><li>每次结果保存到 research/results，含输入配置、UTC时间、模型源码SHA256以及指标；PNG可单独分享。模型迭代后应重新生成证据，不能只保留旧截图。</li></ol>
<h2>实现路径与取舍</h2>
<table><tr><th>路径</th><th>采用条件</th><th>提前处理的成本</th></tr>
<tr><td>当前 Nano + 轻量定位 + NPU检测</td><td>实际完整视觉链路帧龄/内存/温度达到任务要求，航路观测可靠</td><td>相机SDK、算子量化、内存所有权、CPU后处理、采样对时、检测/跟踪分频。建议作为首个可测基线。</td></tr>
<tr><td>阶段性调度：巡航定位为主，近目标视觉为主</td><td>各阶段都保持独立的基本运动状态与失效出口</td><td>切换必须有条件和滞回；不能为省算力关闭避障/状态有效性检查。模型热切换可能导致内存峰值和长暂停。</td></tr>
<tr><td>额外轻量实时节点/辅助核</td><td>有可验证的SDK、时钟和接口</td><td>可卸载采集/时间戳/看门狗，不自动解决 Linux 视觉处理；增加通信、供电、重量和调试成本。</td></tr>
<tr><td>更强伴随计算板</td><td>真实模型转换失败、无法满足帧龄/内存/驱动要求，优化后仍失败</td><td>按完整链路做同输入AB测试；同时核对重量-尺寸、电源瞬态、散热。不能只按TOPS排位采购。</td></tr>
<tr><td>地面/机库计算</td><td>只有正式规则、链路延迟及失联处理都允许才考虑</td><td>当前未确认飞行中持续通信权限；不作为基础姿态/状态估计唯一依赖。</td></tr></table>
<h2>风险与证据清单</h2><p>下表列出模型、接口和硬件实现的风险及对应验证证据。</p>
<table><tr><th>ID</th><th>问题</th><th>状态</th><th>处理路径</th><th>所需证据</th></tr>{rows}</table>
<h2>板上验证与资料</h2>
<p>Nano真实CPU/NPU帧率、摄像头驱动、功耗热设计、场地材料覆盖及最终规则需通过板上测量和正式资料确认。</p>
<p>最新自动检查记录见 <a href="verification.json">verification.json</a>。原始预研PDF与2026-09-09前瞻手册作为资料来源，正式规则发布后需逐条更新。</p>
</body></html>"""
    (ROOT / "docs" / "research" / "index.html").write_text(page, encoding="utf-8")


if __name__ == "__main__":
    main()
