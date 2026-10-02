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
