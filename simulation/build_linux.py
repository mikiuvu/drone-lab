"""Rebuild the real Gazebo plugins and executable companion in existing WSL."""
import json
from pathlib import Path
import subprocess
import shutil

ROOT=Path(__file__).resolve().parent
BASE=Path('/home/devcontainers/drone-sim')


def run(args):subprocess.run(list(map(str,args)),check=True)


def main():
    for source,dest in [('gazebo','build-sensors'),('companion','build-companion')]:
        run(['cmake','-S',ROOT/source,'-B',BASE/dest,'-DCMAKE_BUILD_TYPE=RelWithDebInfo'])
        run(['cmake','--build',BASE/dest,'-j4'])
    run(['ctest','--test-dir',BASE/'build-companion','--output-on-failure'])
    compiler=json.loads((ROOT.parent/'config/toolchain.json').read_text(encoding='utf-8-sig'))['compiler']
    compiler=compiler.removesuffix('-gcc')+'-g++'
    include=BASE/'rv-include';include.mkdir(exist_ok=True)
    shutil.copytree('/usr/include/nlohmann',include/'nlohmann',dirs_exist_ok=True)
    run(['cmake','-S',ROOT/'companion','-B',BASE/'build-companion-rv','-DCMAKE_SYSTEM_NAME=Linux',
        '-DCMAKE_SYSTEM_PROCESSOR=riscv64','-DCMAKE_CXX_COMPILER='+compiler,
        '-DCMAKE_CXX_FLAGS=-I'+str(include),'-DCMAKE_EXE_LINKER_FLAGS=-static','-DCOMPANION_BUILD_TESTS=OFF',
        '-DEigen3_DIR=/usr/share/eigen3/cmake','-Dnlohmann_json_DIR=/usr/share/cmake/nlohmann_json'])
    run(['cmake','--build',BASE/'build-companion-rv','-j4'])
    output=ROOT/'bin/riscv64';output.mkdir(parents=True,exist_ok=True)
    for binary in ('drone_companion_runner','team_coordinator'):
        shutil.copy2(BASE/'build-companion-rv'/binary,output/binary)
        run(['file',output/binary])


if __name__=='__main__':main()
