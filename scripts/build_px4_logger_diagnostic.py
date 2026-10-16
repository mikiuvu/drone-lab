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


def build_diagnostic(sdk, build, output, patch, retention=False, imu_replay=False):
    if imu_replay and not retention:
        raise ValueError('IMU replay retention requires visual odometry retention')
    sdk, build, patch = (p.resolve(strict=True) for p in (sdk, build, patch))
    output = output.resolve()
    if output == sdk or output == build or sdk in output.parents or build in output.parents:
        raise ValueError('输出目录必须在 PX4 源码树外')
    if output.exists() and any(output.iterdir()):
        raise ValueError('输出目录必须为空；已有构建请使用另一个目录')
    commands = json.loads((build / 'compile_commands.json').read_text(encoding='utf-8'))
    output.mkdir(parents=True, exist_ok=True)
    source_root = output / 'source'
    source_names = list(SOURCES)
    copied_names = set(source_names)
    if retention:
        copied_names.add('src/modules/logger/logger.h')
        for path in (sdk / 'platforms/common/uORB').iterdir():
            if path.is_file() and path.suffix in ('.cpp', '.hpp', '.h'):
                copied_names.add(path.relative_to(sdk).as_posix())
        source_names.extend(sorted(row['file'][len(str(sdk)) + 1:] for row in commands
                                   if Path(row['file']).parent == sdk / 'platforms/common/uORB'))
    originals = {}
    for name in sorted(copied_names):
        original = sdk / name
        target = source_root / name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(original, target)
        originals[name] = sha256(original)
    apply_diagnostic_patch(source_root, patch)
    imu_patch = Path(__file__).resolve().parents[1] / 'simulation/px4/imu-replay-log-retention.patch'
    if imu_replay:
        apply_diagnostic_patch(source_root, imu_patch)
    original_archive = build / 'src/modules/logger/libmodules__logger.a'
    archive = output / original_archive.name
    shutil.copy2(original_archive, archive)
    retained_archive = output / 'libuORB.a'
    if retention:
        shutil.copy2(build / 'platforms/common/uORB/libuORB.a', retained_archive)
    compile_commands = []
    for name in source_names:
        original = sdk / name
        entry = next(row for row in commands if Path(row['file']).resolve() == original)
        argv = list(entry['arguments']) if 'arguments' in entry else shlex.split(entry['command'])
        if argv.count('-c') != 1 or argv.count('-o') != 1:
            raise ValueError('无法确定 logger 源码编译参数')
        obj = output / (original.name + '.o')
        argv[argv.index('-c') + 1] = str(source_root / name)
        argv[argv.index('-o') + 1] = str(obj)
        if retention:
            argv[1:1] = ['-I' + str(source_root / 'platforms/common'),
                         '-I' + str(source_root / 'platforms/common/uORB')]
        argv.append('-I' + str(original.parent))
        subprocess.run(argv, cwd=entry['directory'], check=True)
        target_archive = retained_archive if name.startswith('platforms/common/uORB/') else archive
        subprocess.run(['ar', 'r', str(target_archive), str(obj)], check=True)
        compile_commands.append(argv)
    raw = subprocess.run(['ninja', '-t', 'commands', 'bin/px4'], cwd=build,
                         capture_output=True, encoding='utf-8', check=True).stdout.splitlines()[-1]
    binary_dir = output / 'bin'
    binary_dir.mkdir()
    binary = binary_dir / 'px4'
    link = link_command(raw, archive, binary)
    if retention:
        if 'platforms/common/uORB/libuORB.a' not in link:
            raise ValueError('链接步骤缺少 uORB 静态库')
        link = [str(retained_archive) if item == 'platforms/common/uORB/libuORB.a' else item for item in link]
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
        'diagnostic_sources': {name: sha256(source_root / name) for name in sorted(copied_names)},
        'compile_argv': compile_commands, 'link_argv': link,
        'visual_odometry_retention': retention,
        'imu_replay_retention': imu_replay,
    }
    if imu_replay:
        manifest['imu_patch_sha256'] = sha256(imu_patch)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n', encoding='utf-8')
    return manifest


def check_retention_contract(sdk, build, output, manifest, topic='vehicle_visual_odometry'):
    """将原生 uORB 测试链接到基线与候选静态库，分别验证突发发布。"""
    import resource

    test = Path(__file__).resolve().parents[1] / 'simulation/tests/test_px4_odometry_retention.cpp'
    entries = json.loads((build / 'compile_commands.json').read_text(encoding='utf-8'))
    entry = next(row for row in entries if row['file'].endswith('/uORBDeviceNode.cpp'))
    original_main = 'platforms/posix/CMakeFiles/px4.dir/src/px4/common/main.cpp.o'
    renamed_main = output / 'px4-main-renamed.o'
    subprocess.run(['objcopy', '--redefine-sym', 'main=px4_original_main',
                    str(build / original_main), str(renamed_main)], check=True)
    macros = {'vehicle_visual_odometry': None, 'sensor_combined': 'TEST_SENSOR_COMBINED',
              'ekf2_timestamps': 'TEST_EKF2_TIMESTAMPS'}
    if topic not in macros:
        raise ValueError('unsupported retention contract topic')
    suffix = '' if topic == 'vehicle_visual_odometry' else '-' + topic
    results = {'source_sha256': sha256(test), 'topic': topic, 'variants': {}}

    def disable_core():
        resource.setrlimit(resource.RLIMIT_CORE, (0, 0))

    for variant in ('baseline', 'candidate'):
        obj, binary = (output / ('retention-contract-' + variant + suffix + extension)
                       for extension in ('.o', ''))
        argv = list(entry['arguments']) if 'arguments' in entry else shlex.split(entry['command'])
        argv[argv.index('-c') + 1] = str(test)
        argv[argv.index('-o') + 1] = str(obj)
        argv.append('-UNDEBUG')
        if macros[topic]:
            argv.append('-D' + macros[topic])
        if variant == 'candidate':
            argv[1:1] = ['-DRETENTION_CANDIDATE', '-I' + str(output / 'source/platforms/common'),
                         '-I' + str(output / 'source/platforms/common/uORB')]
        subprocess.run(argv, cwd=entry['directory'], check=True)
        link = list(manifest['link_argv'])
        link[link.index('-o') + 1] = str(binary)
        link[link.index(original_main)] = str(renamed_main)
        link.insert(1, str(obj))
        if variant == 'baseline':
            replacements = {str(output / 'libuORB.a'): str(build / 'platforms/common/uORB/libuORB.a'),
                            str(output / 'libmodules__logger.a'): str(build / 'src/modules/logger/libmodules__logger.a')}
            link = [replacements.get(item, item) for item in link]
        subprocess.run(link, cwd=build, check=True)
        result = subprocess.run([str(binary)], capture_output=True, encoding='utf-8',
                                timeout=20, preexec_fn=disable_core)
        results['variants'][variant] = {'exit_code': result.returncode, 'stdout': result.stdout,
                                       'stderr': result.stderr, 'binary_sha256': sha256(binary)}
    (output / ('retention-contract' + suffix + '.json')).write_text(
        json.dumps(results, indent=2) + '\n', encoding='utf-8')
    baseline, candidate = (results['variants'][name] for name in ('baseline', 'candidate'))
    if baseline['exit_code'] != -6 or 'sample.timestamp==expected' not in baseline['stderr']:
        raise RuntimeError('基线没有复现预期的突发发布缺失')
    if candidate['exit_code'] != 0:
        raise RuntimeError('候选原生 uORB 保留历史测试失败')
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk', type=Path, required=True)
    parser.add_argument('--build-dir', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--retain-visual-odometry', action='store_true', help='保留 8 条里程计发布历史供 logger 读取')
    parser.add_argument('--retain-imu-replay', action='store_true', help='为 IMU 和 EKF2 时间索引各保留 16 条发布历史')
    parser.add_argument('--check-retention-contract', action='store_true', help='运行基线与候选的原生 uORB 读取测试')
    args = parser.parse_args()
    if args.check_retention_contract and not args.retain_visual_odometry:
        parser.error('--check-retention-contract requires --retain-visual-odometry')
    if args.retain_imu_replay and not args.retain_visual_odometry:
        parser.error('--retain-imu-replay requires --retain-visual-odometry')
    name = 'visual-odometry-log-retention.patch' if args.retain_visual_odometry else 'logger-subscription-diagnostics.patch'
    patch = Path(__file__).resolve().parents[1] / 'simulation/px4' / name
    manifest = build_diagnostic(args.sdk, args.build_dir or args.sdk / 'build/px4_sitl_default',
                                args.output, patch, retention=args.retain_visual_odometry,
                                imu_replay=args.retain_imu_replay)
    if args.check_retention_contract:
        check_retention_contract(args.sdk.resolve(), Path(manifest['build']), args.output.resolve(), manifest)
        if args.retain_imu_replay:
            for topic in ('sensor_combined', 'ekf2_timestamps'):
                check_retention_contract(args.sdk.resolve(), Path(manifest['build']),
                                         args.output.resolve(), manifest, topic=topic)
    print(json.dumps({k: manifest[k] for k in ('output', 'baseline_binary_sha256', 'diagnostic_binary_sha256')}))


if __name__ == '__main__':
    main()
