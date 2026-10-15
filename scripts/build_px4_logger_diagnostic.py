"""从现有 PX4 SITL 构建生成独立的 logger 订阅诊断程序。"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess


SOURCES = ('src/modules/logger/logged_topics.cpp', 'src/modules/logger/logger.cpp')


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def apply_diagnostic_patch(source_root, patch):
    """只在复制的源码目录应用补丁，不向上寻找工作区仓库。"""
    environment = dict(os.environ)
    for name in ('GIT_DIR', 'GIT_WORK_TREE', 'GIT_COMMON_DIR', 'GIT_INDEX_FILE'):
        environment.pop(name, None)
    environment['GIT_CEILING_DIRECTORIES'] = str(source_root.parent.resolve())
    for arguments in (['--check'], []):
        subprocess.run(['git', '-c', 'core.autocrlf=false', 'apply', *arguments, str(patch)], cwd=source_root,
                       env=environment, check=True)


def link_command(raw, archive, binary):
    """从 Ninja 的复合命令中提取编译器链接步骤，保留参数边界。"""
    segments = [[]]
    for token in shlex.split(raw):
        if token == '&&':
            segments.append([])
        else:
            segments[-1].append(token)
    links = [s for s in segments if s and Path(s[0]).name in ('c++', 'g++', 'clang++') and '-o' in s]
    if len(links) != 1:
        raise ValueError('需要单个可识别的 PX4 链接步骤')
    command = list(links[0])
    if any(t in (';', '|', '>', '<', '||') for t in command):
        raise ValueError('链接步骤含有额外 shell 操作')
    original = 'src/modules/logger/libmodules__logger.a'
    if command.count(original) != 1:
        raise ValueError('链接步骤需要单个 logger 静态库')
    command[command.index(original)] = str(archive)
    command[command.index('-o') + 1] = str(binary)
    return command


def build_diagnostic(sdk, build, output, patch):
    sdk, build, patch = (p.resolve(strict=True) for p in (sdk, build, patch))
    output = output.resolve()
    if output == sdk or output == build or sdk in output.parents or build in output.parents:
        raise ValueError('输出目录必须在 PX4 源码树外')
    if output.exists() and any(output.iterdir()):
        raise ValueError('输出目录必须为空；已有构建请使用另一个目录')
    commands = json.loads((build / 'compile_commands.json').read_text(encoding='utf-8'))
    output.mkdir(parents=True, exist_ok=True)
    source_root = output / 'source'
    originals = {}
    for name in SOURCES:
        original = sdk / name
        target = source_root / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original, target)
        originals[name] = sha256(original)
    apply_diagnostic_patch(source_root, patch)
    original_archive = build / 'src/modules/logger/libmodules__logger.a'
    archive = output / original_archive.name
    shutil.copy2(original_archive, archive)
    compile_commands = []
    for name in SOURCES:
        original = sdk / name
        entry = next(row for row in commands if Path(row['file']).resolve() == original)
        argv = list(entry['arguments']) if 'arguments' in entry else shlex.split(entry['command'])
        if argv.count('-c') != 1 or argv.count('-o') != 1:
            raise ValueError('无法确定 logger 源码编译参数')
        obj = output / (original.name + '.o')
        argv[argv.index('-c') + 1] = str(source_root / name)
        argv[argv.index('-o') + 1] = str(obj)
        argv.append('-I' + str(original.parent))
        subprocess.run(argv, cwd=entry['directory'], check=True)
        subprocess.run(['ar', 'r', str(archive), str(obj)], check=True)
        compile_commands.append(argv)
    raw = subprocess.run(['ninja', '-t', 'commands', 'bin/px4'], cwd=build,
                         capture_output=True, encoding='utf-8', check=True).stdout.splitlines()[-1]
    binary_dir = output / 'bin'
    binary_dir.mkdir()
    binary = binary_dir / 'px4'
    link = link_command(raw, archive, binary)
    subprocess.run(link, cwd=build, check=True)
    for path in (build / 'bin').iterdir():
        if path.name != 'px4':
            (binary_dir / path.name).symlink_to(binary if path.is_symlink() else path)
    for name, digest in originals.items():
        if sha256(sdk / name) != digest:
            raise RuntimeError('构建期间 PX4 源码发生变化')
    manifest = {
        'sdk': str(sdk), 'build': str(build), 'output': str(output),
        'baseline_binary_sha256': sha256(build / 'bin/px4'),
        'diagnostic_binary_sha256': sha256(binary),
        'patch_sha256': sha256(patch), 'original_sources': originals,
        'diagnostic_sources': {name: sha256(source_root / name) for name in SOURCES},
        'compile_argv': compile_commands, 'link_argv': link,
    }
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    patch = Path(__file__).resolve().parents[1] / 'simulation/px4/logger-subscription-diagnostics.patch'
    manifest = build_diagnostic(args.sdk, args.build_dir or args.sdk / 'build/px4_sitl_default', args.output, patch)
    print(json.dumps({k: manifest[k] for k in ('output', 'baseline_binary_sha256', 'diagnostic_binary_sha256')}))


if __name__ == '__main__':
    main()
