"""@file audit_research.py
@brief 检查研究证据文件、本地 HTML 链接和源码摘要的一致性；只读检查，失败以非零退出。
"""

import hashlib
import json
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


class Links(HTMLParser):
    """@brief 收集 HTML 的 a/img 标签中的 href/src，用于本地文件存在性检查。
    """
    def __init__(self):
        """@brief 初始化解析器和链接列表。
        """
        super().__init__()
        self.links = []

    def handle_starttag(self, tag, attrs):
        """@brief 收集链接属性；远端 URL 的排除在 main 中处理。
        """
        if tag in ("a", "img"):
            self.links.extend(value for key, value in attrs if key in ("href", "src"))


def main():
    """@brief 读取证据清单、总览 HTML 和 baseline 摘要，报告缺失或过期来源。
    @note 不重跑模型或修复结果；注释改变也会改变完整源码 SHA256。
    """
    evidence = json.loads(
        (ROOT / "research" / "completion_matrix.json").read_text(encoding="utf-8")
    )
    missing = []
    for requirement in evidence["requirements"]:
        for name in requirement["evidence"]:
            if name.startswith("V1 "):
                continue
            if not (ROOT / name).is_file():
                missing.append(name)
    for path in (ROOT / "docs" / "research").glob("*.html"):
        parser = Links()
        parser.feed(path.read_text(encoding="utf-8"))
        for link in parser.links:
            url = urlsplit(link)
            if url.scheme or url.netloc or not url.path:
                continue
            target = (path.parent / unquote(url.path)).resolve()
            if not target.is_file():
                missing.append(f"{path.name}: {link}")
    bundle = json.loads(
        (ROOT / "research" / "results" / "baseline.json").read_text(encoding="utf-8")
    )
    for name, run in bundle["runs"].items():
        if (
            hashlib.sha256((ROOT / run["source"]).read_bytes()).hexdigest()
            != run["source_sha256"]
        ):
            missing.append(name + ": stale source digest")
    if missing:
        raise SystemExit(chr(10).join(missing))
    print(
        "PASS: requirements evidence files, local report links and current source digests"
    )


if __name__ == "__main__":
    main()
