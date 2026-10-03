"""@file plots.py
@brief 将已计算的研究结果绘成独立 PNG，采用 Agg 画布供无窗口环境使用。
"""

from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg


def save_plot(model, result, path):
    """@brief 绘制调度、几何或时间错配结果，保存到指定路径。
    @param model compute、geometry 或 integration。
    @param result 对应模型的结果字典，直接读取既有指标。
    @param path PNG 输出路径，父目录须已存在；已有文件会被覆盖。
    @return 已保存路径字符串；不支持的模型返回 None。
    @note 不重新计算模型；帧龄图使用对数轴，极小柱高仅用于显示，标注保留实际数值。
    """
    fig = Figure(figsize=(11, 5.5), layout="constrained")
    FigureCanvasAgg(fig)
    if model == "compute":
        metric = result["metrics"]
        left, right = fig.subplots(1, 2)
        names = ["fifo", "latest"]
        age = [metric[n]["frames"]["age_ms"]["p95"] for n in names]
        for i, value in enumerate(age):
            if value is not None:
                left.bar(i, max(value, 0.001), color=["#e57e55", "#168aad"][i])
                left.annotate(
                    f"{value:.1f} ms", (i, max(value, 0.001)), ha="center", va="bottom"
                )
            else:
                left.text(i, 1, "No completed frames", ha="center")
        left.set_xticks([0, 1], ["FIFO", "Latest-frame"])
        left.set_yscale("log")
        left.set_ylabel("Completed frame age p95 (ms, log scale)")
        left.set_title("Freshness includes queue time")
        buffers = [metric[n]["memory"]["max_held_frames"] for n in names]
        right.bar(names, buffers, color=["#e57e55", "#168aad"])
        right.set_ylabel("Peak held image buffers")
        right.set_title("Waiting + in-flight; excludes SDK pools")
        for i, value in enumerate(buffers):
            right.annotate(str(value), (i, value), ha="center", va="bottom")
        fig.suptitle("Synthetic CPU / NPU scheduling")
    elif model == "geometry":
        scenarios = result["scenarios"]
        left, right = fig.subplots(1, 2)
        names = [s["id"] for s in scenarios]
        left.barh(names, [s["hit_count"] for s in scenarios], color="#168aad")
        left.set_xlabel("Ideal zone-center ray hits")
        left.set_title("Synthetic finite-height walls")
        right.barh(names, [s["rank"] for s in scenarios], color="#e59a49")
        right.set_xlim(0, 3.5)
        right.set_xticks([0, 1, 2, 3])
        right.set_xlabel("Local rank for (x, y, yaw), maximum 3")
        right.set_title("Local observability with known association")
        fig.suptitle("Dual ToF observability")
    elif model == "integration":
        axis = fig.subplots()
        values = [
            100 * result["metrics"]["translation_age_error_m"],
            100 * result["metrics"]["rotation_chord_error_m"],
        ]
        axis.bar(
            ["Translation age error", "Rotation point error"],
            values,
            color=["#168aad", "#e59a49"],
        )
        axis.set_ylabel("Centimeters (different error mechanisms; do not sum)")
        for i, value in enumerate(values):
            axis.annotate(f"{value:.2f} cm", (i, value), ha="center", va="bottom")
        fig.suptitle("Constant-velocity timing sensitivity - analytic model")
    else:
        return None
    fig.savefig(path, dpi=130)
    return str(path)
