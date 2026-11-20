#!/usr/bin/env python3
"""Build and run isolated native tests. Requires only Python 3 and clang/cc.

Functional failures return nonzero. Timing diagnostics are always displayed but
only fail the run with --strict-timing. Nothing is flashed or committed. Builds
and generated SDK/fixture headers live in an automatically removed temp folder.
"""
import argparse
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile
sys.dont_write_bytecode = True
from pio_model import Pio

ROOT = Path(__file__).resolve().parents[1]

def generate_headers(build, pianolas):
    # Compile the actual C initialisers embedded in the .pio files too. The
    # instruction interpreter separately executes their assembly above %c-sdk.
    for name in ('pianolaPinDriver', 'pianola24bitClock'):
        source = (ROOT / f'{name}.pio').read_text()
        import re
        m = re.search(r'%\s*c-sdk\s*\{(.*?)%\}', source, re.S)
        if not m:
            raise RuntimeError(f'{name}: cannot find embedded C SDK initialiser')
        (build / f'{name}.pio.h').write_text(m[1])
    records = []
    paths = sorted(pianolas.glob('*.txt'))
    if not paths:
        raise RuntimeError(f'No production .txt fixtures in {pianolas}; use --pianolas PATH')
    # Independent protocol conversion: positive round-to-nearest, 2.56 us/tick.
    # Test a few exposure values for the -1 placeholder, not just one fixture.
    for path in paths:
        for exposure in (0.001, 0.010, 0.100) if '-1' in path.read_text() else (0.010,):
            entries, endless = [], 0
            for raw in path.read_text().splitlines():
                line = raw.split('#', 1)[0].strip()
                if not line:
                    continue
                if line.startswith('Endless'):
                    endless = int(line.split()[1])
                elif line.startswith('Delay to frame capture'):
                    continue
                else:
                    mask, duration = line.split()
                    seconds = float(duration)
                    seconds = exposure if seconds == -1 else seconds
                    if seconds < 0:
                        raise ValueError(f'{path}: invalid negative duration')
                    entries.append((int(mask, 0), math.floor(seconds / 2.56e-6 + 0.5)))
            if not 1 <= len(entries) <= 255:
                raise ValueError(f'{path}: fixture needs 1..255 entries (plus host sentinel)')
            name = f'{path.name} (exposure {exposure * 1000:g} ms)'
            records.append((name, endless, entries))
    header = ['typedef struct { unsigned mask; uint32_t duration; } fixture_entry;',
              'typedef struct { const char *name; unsigned endless,count; fixture_entry entries[256]; } fixture;',
              'static const fixture fixtures[] = {']
    for name, endless, entries in records:
        values = ','.join('{%d,%d}' % pair for pair in entries)
        header.append('{%s,%d,%d,{%s}},' % (json.dumps(name), endless, len(entries), values))
    header.append('};')
    (build / 'fixtures.h').write_text('\n'.join(header))

def pio_tests(binary):
    def driver():
        words = json.loads(subprocess.check_output([str(binary), '--trace'], text=True))['words']
        p = Pio(ROOT / 'pianolaPinDriver.pio')
        # Model a real four-word FIFO, initially full. This test deliberately
        # removes CPU starvation so it isolates the assembly's own behaviour.
        assert len(words) == 4, 'Expected two actual firmware-generated entries'
        p.tx.extend(words)
        return p
    def no_glitch():
        p = driver(); p.until(lambda: not p.tx and len(p.writes) >= 3)
        # Adjacent entries both assert the same output: no low is permissible
        # between their OUT instructions, regardless of final-state policy.
        values = [v for _, v in p.writes]
        assert values[:3] != [1, 0, 1], 'PIO clears output between adjacent high entries; continuous high was required'
    def final_hold():
        p = driver(); p.until(lambda: not p.tx and len(p.writes) >= 2)
        for _ in range(1000): p.step()
        assert p.pins == 1, f'Final output should remain high; actual modelled pattern is 0x{p.pins:x}'
    def four_tick_width():
        p = driver(); p.until(lambda: len(p.writes) >= 2)
        elapsed = p.writes[1][0] - p.writes[0][0]
        assert elapsed == 128, f'4-tick boundary: expected 128 PIO cycles (10.24 us); first following write is at {elapsed} cycles ({elapsed * .08:.2f} us)'
    def clock_freshness():
        p = Pio(ROOT / 'pianola24bitClock.pio'); p.tx.append(15)
        p.until(lambda: len(p.rx) == 4)
        p.rx.popleft()  # CPU reads one timestamp, then drains request FIFO.
        p.tx.clear()
        for _ in range(100): p.step()
        request_cycle = p.cycles; p.tx.append(15)
        p.until(lambda: bool(p.rx))
        sampled_cycle, _ = p.rx.popleft()
        assert sampled_cycle >= request_cycle, f'Clock returned sample from cycle {sampled_cycle} for request at cycle {request_cycle}: stale RX data'
    def divider():
        import re
        source = (ROOT / 'picoTimingBoxFirmware.c').read_text()
        def divisor(name):
            return int(re.search(rf'{name}\s*=\s*(0x[0-9a-fA-F]+)', source)[1], 16) / 256
        clock = divisor('pianolaClockDivisorFP') * 10
        output = divisor('pinDriverClockDivisorFP') * 32
        ppm = (output / clock - 1) * 1e6
        assert abs(ppm) < 1, f'Clock and output ticks differ by {ppm:.2f} ppm (configured divisors; absolute oscillator error not modelled)'
    return [
        ('functional', 'pio_adjacent_patterns', 'PIO / no inserted low between high entries', no_glitch),
        ('functional', 'pio_final_pattern', 'PIO / retain final output pattern', final_hold),
        ('functional', 'clock_sample_freshness', 'Clock PIO / fresh sample per request', clock_freshness),
        ('timing', 'pio_four_tick_duration', 'Timing / four-tick assembly duration', four_tick_width),
        ('timing', 'clock_divider_agreement', 'Timing / clock/output tick agreement', divider),
    ]

def model_self_test():
    # Independent short instruction checks guard against an interpreter that
    # mistakenly blesses broken assembly. Not an exhaustive emulator validation.
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / 'check.pio'
        path.write_text('.program check\n.wrap_target\nset x, 1\nloop:\njmp x-- loop [2]\nmov pins, ~null\nmov pins, null\n.wrap\n')
        p = Pio(path, out_count=5)
        for _ in range(5): p.step()
        assert p.writes == [(7, 31), (8, 0)], p.writes
        assert p.reg['x'] == 0xFFFFFFFF, 'JMP post-decrement semantics incorrect'


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--strict-timing', action='store_true', help='also return failure for timing diagnostics')
    ap.add_argument('--filter', default='', help='run tests whose name or area contains this text')
    ap.add_argument('--list', action='store_true', help='list tests without executing them')
    ap.add_argument('--pianolas', type=Path, default=ROOT.parent / 'Pianolas', help='production fixture directory')
    ap.add_argument('--no-sanitize', action='store_true', help='disable ASan/UBSan (enabled by default)')
    args = ap.parse_args()
    cc = shlex.split(os.environ.get('CC', 'clang'))
    if not shutil.which(cc[0]):
        ap.error(f'Compiler {cc[0]!r} not found; install Xcode command line tools or set CC')
    model_self_test()
    with tempfile.TemporaryDirectory(prefix='timingbox-native-') as d:
        build = Path(d); generate_headers(build, args.pianolas)
        binary = build / 'firmware_tests'
        command = cc + ['-std=gnu2x', '-O1', '-g', '-Wno-unused-function', '-Wno-infinite-recursion',
                        '-I', str(build), '-I', str(ROOT / 'tests/native'),
                        str(ROOT / 'tests/native/test_firmware.c'), '-lm', '-o', str(binary)]
        if not args.no_sanitize:
            command += ['-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-sanitize-recover=all']
        built = subprocess.run(command, text=True, capture_output=True)
        if built.returncode:
            print('BUILD ERROR (tests were not run)\n' + built.stdout + built.stderr, file=sys.stderr)
            return 2
        if built.stderr:
            print(built.stderr, file=sys.stderr)
        tests = []
        # Isolated subprocess per test; timeouts are functional failures.
        env = os.environ.copy()
        env['ASAN_OPTIONS'] = 'detect_leaks=0'  # Process-lifetime queues; trapped blocking reads can abandon allocations.
        def native(name):
            def check():
                try:
                    result = subprocess.run([str(binary), name], text=True, capture_output=True, timeout=5, env=env)
                except subprocess.TimeoutExpired:
                    raise AssertionError('Five-second watchdog expired: possible deadlock/unbounded loop') from None
                assert result.returncode == 0, (result.stdout + result.stderr).strip() or f'Exited with status {result.returncode}'
            return check
        for line in subprocess.check_output([str(binary), '--list'], text=True, env=env).splitlines():
            name, area = line.split('\t'); tests.append(('timing' if name.startswith('timing_') else 'functional', name, area, native(name)))
        tests += pio_tests(binary)
        tests = [t for t in tests if args.filter.lower() in (t[1] + ' ' + t[2]).lower()]
        if not tests:
            ap.error('No tests match the filter')
        if args.list:
            for category, name, area, _ in tests: print(f'{category:10} {name:28} {area}')
            return 0
        print('Actual firmware C + strict fake peripherals; actual PIO source + limited instruction model.')
        print('No Pico required. CPU performance, bus contention, multicore interleavings and electrical timing are NOT measured.\n')
        failures = timings = passes = 0
        for category, name, area, check in tests:
            try:
                check(); passes += 1
                print(f'PASS  [{category.upper()}] {name} — {area}')
            except (AssertionError, ValueError, KeyError) as e:
                if category == 'timing': timings += 1; label = 'WARN'
                else: failures += 1; label = 'FAIL'
                print(f'{label}  [{category.upper()}] {name} — {area}')
                for line in str(e).splitlines(): print('      ' + line)
        print(f'\n{passes} passed; {failures} functional failures; {timings} timing warnings.')
        print('Timing warnings are separate from functional failures; --strict-timing makes them affect exit status.')
        return 1 if failures or (args.strict_timing and timings) else 0

if __name__ == '__main__':
    sys.exit(main())
