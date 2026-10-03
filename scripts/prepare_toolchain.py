"""@file prepare_toolchain.py
@brief 将现有 RISC-V 工具链打包并部署到 WSL，按 Git 索引恢复符号链接。
@note 导入即执行：覆盖 downloads/riscv-toolchain-local.tar、解包到 WSL 用户目录，并重写 config/toolchain.json；不下载工具链。
"""

import json
import pathlib
import subprocess
import tarfile

ROOT = pathlib.Path(__file__).resolve().parents[1]
repo = ROOT / "tools" / "host-tools"
prefix = "gcc/riscv64-linux-musl-x86_64"
raw = subprocess.run(
    ["git", "-C", str(repo), "ls-files", "--stage", "-z", "--", prefix],
    check=True,
    capture_output=True,
).stdout
# Git 的 120000 模式识别符号链接，Windows 工作树可能将其保存为链接目标文本。
links = {}
for entry in raw.split(bytes([0])):
    if not entry:
        continue
    info, name = entry.decode().split(chr(9), 1)
    if info.startswith("120000"):
        links[name] = (repo / name).read_text(encoding="utf-8").strip()
archive = ROOT / "downloads" / "riscv-toolchain-local.tar"
# tar 恢复真实链接和可执行权限，使编译器在 WSL 中能找到辅助程序。
with tarfile.open(archive, "w") as tar:
    for path in (repo / prefix).rglob("*"):
        name = path.relative_to(repo).as_posix()
        if name in links:
            info = tarfile.TarInfo(name)
            info.type = tarfile.SYMTYPE
            info.linkname = links[name]
            info.mode = 0o777
            tar.addfile(info)
        else:
            info = tar.gettarinfo(str(path), name)
            info.mode = (
                0o755
                if path.is_dir() or "/bin/" in name or "/libexec/" in name
                else 0o644
            )
            if path.is_file():
                with path.open("rb") as f:
                    tar.addfile(info, f)
            else:
                tar.addfile(info)
wsl = ["wsl", "-d", "Ubuntu-22.04", "--exec"]
home = (
    subprocess.run(wsl + ["printenv", "HOME"], check=True, capture_output=True)
    .stdout.decode()
    .strip()
)
destination = home + "/.local/share/drone-lab-toolchain"
subprocess.run(wsl + ["mkdir", "-p", destination], check=True)
linux_archive = (
    subprocess.run(
        wsl + ["wslpath", "-a", str(archive)], check=True, capture_output=True
    )
    .stdout.decode()
    .strip()
)
subprocess.run(wsl + ["tar", "-xf", linux_archive, "-C", destination], check=True)
(ROOT / "config" / "toolchain.json").write_text(
    json.dumps(
        {
            "compiler": destination
            + "/"
            + prefix
            + "/bin/riscv64-unknown-linux-musl-gcc"
        },
        indent=2,
    ),
    encoding="utf-8",
)
print("Toolchain ready:", destination, "symlinks:", len(links))
