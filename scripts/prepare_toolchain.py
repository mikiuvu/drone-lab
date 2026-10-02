"""Preserve Git symlinks when transferring the existing toolchain into WSL."""

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
links = {}
for entry in raw.split(bytes([0])):
    if not entry:
        continue
    info, name = entry.decode().split(chr(9), 1)
    if info.startswith("120000"):
        links[name] = (repo / name).read_text(encoding="utf-8").strip()
archive = ROOT / "downloads" / "riscv-toolchain-local.tar"
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
