"""Check the research deliverable against current source and local evidence."""

import hashlib
import json
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[1]


class Links(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []

    def handle_starttag(self, tag, attrs):
        if tag in ("a", "img"):
            self.links.extend(value for key, value in attrs if key in ("href", "src"))


def main():
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
