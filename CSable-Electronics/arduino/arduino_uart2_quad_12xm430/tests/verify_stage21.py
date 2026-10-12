"""Host-only regression runner; never uploads or changes production calibration.

Requires Python 3, git, a C++17 compiler, and Dynamixel2Arduino 0.8.2 sources.
Generated includes/executables live in --build-dir (outside the sketch).
"""
from pathlib import Path
import argparse
import os
import re
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('--library', type=Path, required=True, help='Dynamixel2Arduino library directory')
parser.add_argument('--compiler', type=Path, required=True, help='clang++, g++, or zig executable')
parser.add_argument('--build-dir', type=Path, default=Path(tempfile.gettempdir()) / 'quad_nav_stage21_tests')
args = parser.parse_args()
here = Path(__file__).resolve().parent
root = Path(subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], cwd=here, text=True).strip())
sketch = here.parent / 'arduino_uart2_quad_12xm430.ino'
rel = sketch.relative_to(root).as_posix()
a = subprocess.check_output(['git', 'show', '3e7472c:' + rel], cwd=root).decode('utf-8').replace('\r\n', '\n')
b = sketch.read_text(encoding='utf-8')


def function(source, name):
    match = re.search(r'^\w[\w *&]*\b' + name + r'\([^\n]*\)\s*\{', source, re.M)
    assert match, name
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[match.start():pos]


names = ['inverseKinematics', 'forwardKinematics', 'getCycleReferenceByIndex', 'computeAction',
         'verifyAllReferences', 'setAllLegActions', 'writeAllLegsRad', 'jointRadToMotorRaw',
         'motorRawToJointRad', 'sendImuData', 'uppercaseInPlace', 'handleCommand',
         'toggleWalkStand', 'pollCommandSerial']
for name in names:
    assert function(a, name) == function(b, name), name
for name in ['FR_THIGH', 'FR_CRANK', 'FL_THIGH', 'FL_CRANK', 'RR_THIGH', 'RR_CRANK', 'RL_THIGH', 'RL_CRANK']:
    pattern = r'^.*\{"' + name + r'".*$'
    assert re.search(pattern, a, re.M).group() == re.search(pattern, b, re.M).group(), name
for start, end in [('const float PI_F', '// Motor types and configuration'),
                   ('const float L2_DG', '// Runtime state')]:
    assert a[a.index(start):a.index(end)] == b[b.index(start):b.index(end)]
for source in [a, b]:
    assert 'const float TS = 0.01f;' in source
start = '    if (mode == MODE_WALKING) {'
end = '    writeAllLegsRad();'
old_loop, new_loop = function(a, 'loop'), function(b, 'loop')
assert old_loop[old_loop.index(start):old_loop.index(end)] == new_loop[new_loop.index(start):new_loop.index(end)]
print('PASS: 14 unchanged functions, eight exact mappings, geometry/gait constants and loop trajectory block', flush=True)

out = args.build_dir.resolve()
assert not out.is_relative_to(here.parent), 'Keep generated fixtures outside the production sketch'
out.mkdir(parents=True, exist_ok=True)
clean = lambda s: re.sub(r'^#include[^\n]*\n', '', s, flags=re.M)
test_source = clean(b).replace('const ZAxisCalibration zCalibration', 'ZAxisCalibration zCalibration')
test_source = test_source.replace('const LegStartupCalibration legStartup', 'LegStartupCalibration legStartup')
test_source = test_source.replace('const bool MOTION_COMMISSIONING_VERIFIED', 'bool MOTION_COMMISSIONING_VERIFIED')
(out / 'stage1.inc').write_text(clean(a), encoding='utf-8')
(out / 'stage2.inc').write_text(test_source, encoding='utf-8')
library = args.library.resolve()
assert 'version=0.8.2' in (library / 'library.properties').read_text(), 'Regression pins the audited decoder version'
source = (library / 'src/Dynamixel2Arduino.cpp').read_text()
# Compile the actual function body verbatim, with a simulated port/read transport.
start = source.index('int32_t Dynamixel2Arduino::readControlTableItem(uint16_t')
end = source.index('\nbool Dynamixel2Arduino::writeControlTableItem(uint16_t', start)
(out / 'library_decode.inc').write_text(source[start:end], encoding='utf-8')
# Compile the actual library control-table implementation as well.
(out / 'Arduino.h').write_text('#pragma once\n#include <cstdint>\n#include <cstring>\n'
                               '#define pgm_read_byte(p) (*(const uint8_t*)(p))\n'
                               '#define pgm_read_word(p) (*(const uint16_t*)(p))\n', encoding='utf-8')
command = [str(args.compiler.resolve())]
if args.compiler.stem.lower() == 'zig':
    command += ['c++']
    os.environ.setdefault('ZIG_GLOBAL_CACHE_DIR', str(out / 'zig-global'))
    os.environ.setdefault('ZIG_LOCAL_CACHE_DIR', str(out / 'zig-local'))
exe = out / ('verify_stage21.exe' if os.name == 'nt' else 'verify_stage21')
command += ['-std=c++17', '-O0', '-Wall', '-Wextra', '-I' + str(out), '-I' + str(library / 'src'),
            str(here / 'verify_stage21.cpp'), str(library / 'src/actuator.cpp'), '-o', str(exe)]
subprocess.run(command, check=True)
subprocess.run([str(exe)], check=True)
