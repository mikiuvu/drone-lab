"""@file make_samples.py
@brief 生成 200 个 0.1 s 步进的演示时刻，覆盖 samples/demo.jsonl。
@note 导入即执行写入；来源是 app.data.demonstration 的合成数据，record_time_s 为相对秒数。
"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from app.data import demonstration

with (ROOT / "samples" / "demo.jsonl").open("w", encoding="utf-8") as f:
    for frame in range(200):
        for item in demonstration(frame / 10, frame):
            item["record_time_s"] = frame / 10
            f.write(json.dumps(item, ensure_ascii=False) + chr(10))
print("Created 20-second sample recording")
