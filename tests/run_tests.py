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
        program = re.search(r'^\.program\s+(\w+)', source, re.M)[1]
        wrap_target = Pio(ROOT / f'{name}.pio').wrap_target
        (build / f'{name}.pio.h').write_text(f'#define {program}_wrap_target {wrap_target}\n' + m[1])
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
    def driver(repeat=False):
        trace = '--repeat-trace' if repeat else '--trace'
        try:
            result = subprocess.run([str(binary), trace], text=True, capture_output=True, timeout=5)
        except subprocess.TimeoutExpired:
            raise AssertionError('Firmware FIFO trace exceeded the five-second watchdog') from None
        assert result.returncode == 0, result.stderr.strip() or f'Trace exited with status {result.returncode}'
        words = json.loads(result.stdout)['words']
        masks = [1, 0x201, 0x200]
        if repeat:
            masks.extend([0x201, 0x200])
        assert len(words) == 2 * len(masks), 'Expected one firmware-generated FIFO pair per entry'

        p = Pio(ROOT / 'pianolaPinDriver.pio')
        # Start with the first pair and refill as soon as the FIFO empties to
        # isolate instruction timing from the CPU's real servicing latency.
        for i in range(0, len(words), 2):
            p.until(lambda: not p.tx)
            p.tx.extend(words[i:i + 2])
        p.until(lambda: not p.tx and p.pins == masks[-1])
        return p, masks

    def no_glitch():
        for repeat in (False, True):
            p, masks = driver(repeat)
            actual = [mask for _, mask in p.writes]
            assert actual == masks, f'Expected direct mask transitions {masks}, got {actual}'

    def final_hold():
        p, masks = driver()
        for _ in range(1000):
            p.step()
        assert [mask for _, mask in p.writes] == masks
        assert p.pins == masks[-1], f'Final mask changed to 0x{p.pins:x}'

    def entry_spacing(repeat=False):
        p, _ = driver(repeat)
        cycles = [cycle - p.writes[0][0] for cycle, _ in p.writes]
        expected = [0, 32, 160, 192, 320] if repeat else [0, 32, 160]
        assert cycles == expected, f'Expected entry writes at {expected} PIO cycles, got {cycles}'

    def final_duration():
        p, _ = driver()
        p.until(lambda: p.pc == p.wrap_target)
        elapsed = p.cycles - p.writes[-1][0]
        assert 32 <= elapsed <= 34, (
            f'Expected idle immediately after the minimum one-tick entry, got {elapsed} PIO cycles'
        )

    def clock_freshness():
        p = Pio(ROOT / 'pianola24bitClock.pio')
        for request in (1, 2):
            request_cycle = p.cycles
            p.tx.append(0xF)
            # Delay the CPU read long enough to expose repeated responses.
            for _ in range(100):
                p.step()
            assert len(p.rx) == 1, f'Request {request} produced {len(p.rx)} queued samples'
            assert not p.tx, f'Clock did not consume request {request}'
            sampled_cycle, _ = p.rx.popleft()
            assert sampled_cycle >= request_cycle, (
                f'Request {request} at cycle {request_cycle} returned a sample from cycle {sampled_cycle}'
            )

    def clock_tick_cycles():
        for counter in (0, 1, 2, 0xFFFFFF):
            for requested in (False, True):
                p = Pio(ROOT / 'pianola24bitClock.pio')
                p.pc = p.labels['wait1']
                p.reg['x'] = counter
                if requested:
                    p.tx.append(0xF)
                # Exercise consecutive samples as well as entry into/exit from
                # wrap, so handling a request cannot slow or freeze the clock.
                for tick in range(4):
                    current = p.reg['x']
                    start = p.cycles
                    p.step()
                    p.until(lambda: p.pc == p.labels['wait1'])
                    context = f'counter={counter}, request={requested}, tick={tick}'
                    assert p.cycles - start == 12, f'{context}: tick took {p.cycles - start} cycles'
                    expected = (current - 1) & 0xFFFFFF
                    assert p.reg['x'] == expected, f'{context}: next counter was {p.reg["x"]:#x}'
                    samples = [sample for _, sample in p.rx]
                    assert samples == ([current] if requested else []), f'{context}: got samples {samples}'
                    p.rx.clear()
                    if requested:
                        p.tx.append(0xF)

    def clock_request_phases():
        # Arriving after MOV STATUS must wait for the next tick. Exercise every
        # instruction boundary of both a normal tick and the reload path, with
        # absolute tick deadlines and independently expected timestamp values.
        for counter in (0, 1, 0xFFFFFF):
            probe = Pio(ROOT / 'pianola24bitClock.pio')
            probe.pc = probe.labels['wait1']
            probe.reg['x'] = counter
            phases = []
            for _ in range(32):
                if phases and probe.pc == probe.labels['wait1']:
                    break
                phases.append(probe.cycles)
                probe.step()
            else:
                raise AssertionError(f'counter={counter}: clock did not return to its tick boundary')
            for arrival in phases:
                p = Pio(ROOT / 'pianola24bitClock.pio')
                p.pc = p.labels['wait1']
                p.reg['x'] = counter
                p.until(lambda: p.cycles == arrival)
                p.tx.append(0xF)
                for tick in range(3):
                    p.step()
                    p.until(lambda: p.pc == p.labels['wait1'])
                    assert p.cycles == (tick + 1) * 12, (
                        f'counter={counter}, arrival={arrival}: tick ended at {p.cycles} cycles'
                    )
                samples = [sample for _, sample in p.rx]
                expected = counter if arrival == 0 else (counter - 1) & 0xFFFFFF
                assert samples == [expected], (
                    f'counter={counter}, arrival={arrival}: expected one sample {expected:#x}, got {samples}'
                )
                assert not p.tx, f'Request at cycle {arrival} was not consumed'

    def divider():
        import re
        source = (ROOT / 'picoTimingBoxFirmware.c').read_text()
        def divisor(name):
            return int(re.search(rf'{name}\s*=\s*(0x[0-9a-fA-F]+)', source)[1], 16) / 256
        clock = divisor('pianolaClockDivisorFP') * 12
        output = divisor('pinDriverClockDivisorFP') * 32
        ppm = (output / clock - 1) * 1e6
        assert clock == output == 384, (
            f'Expected both ticks to take exactly 384 system-clock cycles, got clock={clock:g}, '
            f'output={output:g} ({ppm:.2f} ppm difference; absolute oscillator error not modelled)'
        )
    return [
        ('functional', 'pio_adjacent_patterns', 'PIO / no inserted low between high entries', no_glitch),
        ('functional', 'pio_final_pattern', 'PIO / retain final output pattern', final_hold),
        ('functional', 'clock_sample_freshness', 'Clock PIO / fresh sample per request', clock_freshness),
        ('functional', 'clock_tick_cycles', 'Clock PIO / twelve-cycle ticks through wraparound and consecutive samples', clock_tick_cycles),
        ('functional', 'clock_request_phases', 'Clock PIO / request arrival at each normal/wrap instruction boundary', clock_request_phases),
        ('timing', 'pio_entry_spacing', 'Timing / one- and four-tick entry spacing', entry_spacing),
        ('timing', 'pio_repeat_spacing', 'Timing / entry spacing through the repeat boundary', lambda: entry_spacing(repeat=True)),
        ('timing', 'pio_final_duration', 'Timing / minimum-duration final entry reaches idle after one tick', final_duration),
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
