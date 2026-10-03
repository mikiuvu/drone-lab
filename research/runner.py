"""离线研究统一入口：执行指定模型，保存输入、输出和源码摘要。

界面通过独立解释器启动本模块；execute 读取模型源码字节并计算 SHA256，
随后执行同一份字节，确保结果文件记录的摘要对应实际运行代码。
源码修改后摘要会改变，即使只修改注释；历史结果仍保留其原始摘要。
"""

import argparse
import hashlib
import json
import sys
import types
from datetime import datetime, timezone
from pathlib import Path

FUNCTIONS = {
    "bandwidth": "run_bandwidth",
    "compute": "run_compute",
    "geometry": "run_geometry",
    "integration": "run_integration",
    "control_policy": "run_control",
}


def execute(model, config, output, plot=True):
    """执行一个研究模型并写入结果 JSON。

    model 为 FUNCTIONS 中的名称；config 是模型输入参数字典；output 是结果路径。
    plot 决定是否附带 PNG。绘图失败保存在 plot_error，数值结果仍可写出。
    返回保存的结果字典；模型运行错误或执行期间源码变化会向调用者抛异常。
    """
    if model not in FUNCTIONS:
        raise ValueError("Unknown research model")
    source = Path(__file__).resolve().parent / (model + ".py")
    source_bytes = source.read_bytes()
    before = hashlib.sha256(source_bytes).hexdigest()
    # Compile exactly the hashed bytes: also avoids same-mtime/size .pyc caches.
    module_name = "research." + model
    module = types.ModuleType(module_name)
    module.__file__ = str(source)
    module.__package__ = "research"
    sys.modules[module_name] = module
    exec(compile(source_bytes, str(source), "exec"), module.__dict__)
    result = getattr(module, FUNCTIONS[model])(config)
    if hashlib.sha256(source.read_bytes()).hexdigest() != before:
        raise RuntimeError("Model source changed during simulation; run again")
    payload = {
        "run_utc": datetime.now(timezone.utc).isoformat(),
        "model": model,
        "source_sha256": before,
        "input_config": config,
        "result": result,
        "evidence_type": "offline_simulation",
        "python": sys.version,
        "plot_error": None,
    }
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if plot:
        try:
            from research.plots import save_plot

            payload["plot_path"] = save_plot(model, result, output.with_suffix(".png"))
        except Exception as exc:
            payload["plot_error"] = str(exc)
    output.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False),
        encoding="utf-8",
    )
    return payload


def main():
    """解析命令行模型名、JSON 配置与输出路径，交给 execute 执行。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=FUNCTIONS, required=True)
    parser.add_argument("--config-json", default="{}")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    execute(args.model, json.loads(args.config_json), args.output)


if __name__ == "__main__":
    main()
