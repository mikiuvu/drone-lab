"""@file verify_research.py
@brief 串联离线测试、WSL 原生检查、Nano 交叉编译与研究报告生成。
@note 会更新构建产物、研究结果、图表、总览和 docs/research/verification.json；不是只读检查。
"""

import hashlib
import json
import os
from pathlib import Path
import subprocess
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parents[1]
PYTHON = ROOT / ".venv" / "Scripts" / "python.exe"


def main():
    """@brief 按顺序运行检查并保存退出码、输出末尾与源码/产物摘要。
    @note 每条命令限时 180 s；非零退出先保存失败记录再结束，成功后写入离线检查状态。
    """
    result = {
        "verified_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "PC/offline research, collector and native build checks",
        "checks": [],
        "hardware_verified": False,
    }
    commands = [
        ["-m", "pytest", "-q"],
        ["scripts/check_collector.py"],
        ["scripts/check_native.py"],
        ["scripts/build_nano.py"],
        ["scripts/run_research.py"],
        ["scripts/build_research_report.py"],
    ]
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    for args in commands:
        process = subprocess.run(
            [str(PYTHON)] + args, cwd=ROOT, capture_output=True, env=env, timeout=180
        )
        output = (
            (process.stdout + process.stderr)
            .decode("utf-8", "replace")
            .replace(chr(0), "")
        )
        result["checks"].append(
            {
                "command": args,
                "exit_code": process.returncode,
                "output": output[-16000:],
            }
        )
        if process.returncode:
            result["status"] = "failed"
            (ROOT / "docs" / "research" / "verification.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            raise SystemExit("Check failed: " + " ".join(args))
        print("PASS " + " ".join(args), flush=True)
    paths = [
        *list((ROOT / "research").glob("*.py")),
        ROOT / "nano" / "main.c",
        ROOT / "nano" / "platform.c",
        ROOT / "nano" / "platform.h",
        ROOT / "app" / "research_panel.py",
        ROOT / "build" / "nano-riscv64" / "drone-nano",
    ]
    result["sha256"] = {
        p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in paths
    }
    result["status"] = "passed_offline_checks"
    (ROOT / "docs" / "research" / "verification.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )


if __name__ == "__main__":
    main()
