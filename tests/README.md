# Native firmware tests

Run from the TimingBoxPico repository:

```sh
python3 tests/run_tests.py
```

Or from its parent spim-interface directory:

```sh
python3 TimingBoxPico/tests/run_tests.py
```

Requires Python 3.9+ and Clang with C2x support (the Xcode command line tools on
recent macOS). No Pico SDK, Python packages, USB connection or logic analyser is
needed. `CC` may select another compatible compiler. Builds use temporary folders
and do not change firmware binaries or flash devices.

The production fixtures are read from the sibling `../Pianolas` directory. If
this repository is checked out elsewhere, point at those files explicitly:

```sh
python3 tests/run_tests.py --pianolas /path/to/spim-interface/Pianolas
```

## Reading results

Each test prints its category, stable name, area and a useful explanation on
failure. Tests run independently; one blocked operation or sanitizer failure
cannot prevent the other tests from running.

- `PASS [FUNCTIONAL]`: the particular behavioural check passed. It does not mean
  physical timing has been measured.
- `FAIL [FUNCTIONAL]`: incorrect behaviour, a violated safety/transport contract,
  a sanitizer error, or a five-second watchdog expiry. These make the command
  return exit status 1.
- `PASS [TIMING]`: the stated timing calculation/model check passed.
- `WARN [TIMING]`: duration error, clock mismatch or starvation under the stated
  artificial servicing schedule. These are shown separately and do not affect
  exit status unless `--strict-timing` is supplied. They are not all necessarily
  harmless in an experiment; this classification separates timing fidelity from
  failures such as deadlock, lost commands or incorrect patterns.
- Build/setup errors return status 2 and are not reported as firmware failures.

For example:

```text
FAIL [FUNCTIONAL] stop_queued — Control / stop cancels queued successor
     Stop left a successor in PIO FIFO; it can change outputs after stop
WARN [TIMING] pio_four_tick_duration — Timing / four-tick assembly duration
     4-tick boundary: expected 128 PIO cycles (10.24 us); ...
```

Existing firmware bugs are deliberately ordinary failing tests, not expected
failures which turn the suite green. Fixing them should turn their tests green.
No firmware bug fixes are bundled with the harness. Intentional limitations,
such as fixed output-source mappings and PIV-enable being a no-op, have explicit
compatibility tests. If those features are implemented, update that documented
contract and its tests together.

An inserted low between two high patterns and failure to retain the final pattern
are functional failures: they change the output pattern. Extra elapsed time with
otherwise correct pattern ordering is reported in the timing group.

Useful commands:

```sh
python3 tests/run_tests.py --list
python3 tests/run_tests.py --filter protocol
python3 tests/run_tests.py --filter restart_running
python3 tests/run_tests.py --filter timing
python3 tests/run_tests.py --strict-timing
python3 tests/run_tests.py --no-sanitize
```

Filters match test names and area descriptions, case-insensitively. Use an exact
test name to rerun one failure. Sanitizers (AddressSanitizer and UBSan) are enabled
by default; disable them only when diagnosing toolchain/runtime issues. Leak
checking is disabled because queues live for the process lifetime and a trapped
blocking call may abandon an allocation. Firmware allocations are initially
poisoned with `0xA5`, making uninitialised-output defects reproducible without
MemorySanitizer. Poisoning does not itself establish that all uninitialised reads
have been detected.

## What is actually executed

`native/test_firmware.c` includes the **actual firmware translation unit**. A
small production refactor exposes `pianolaManagerStep()`; the device loop calls
that same step continuously. The host-test macro substitutes includes and renames
the firmware entry point only. The parser and playback logic are not rewritten
or extracted as test-specific copies.

`native/pico_shim.h` supplies the peripheral boundary:

- A manually controlled 24-bit clock for functional scheduling tests.
- A real-capacity four-word fake output FIFO. Submission and consumption are
  separate, and an active entry is separate from a queued successor.
- SDK-style queues that copy complete records and reject writes when full.
- UART byte input and captured output. Tests can present complete packets,
  command bursts or incomplete packets. A read with no available byte raises a
  diagnostic rather than hanging. This does not pretend the device automatically
  has a timeout.
- GPIO/PIO configuration recording and range validation. UART calls record the
  simulated core context; this checks ownership, not real concurrent execution.

The generated native PIO headers contain the **actual C SDK initialisers embedded
in the .pio files**. The real firmware startup runs up to core launch; its settings run against the
strict shim, which would
reject a SET width greater than five. The current pin-configuration test also
checks that unrelated GPIOs are not assigned or driven.

`pio_model.py` interprets the actual assembly in the .pio files, including branch
post-decrement, instruction delays, FIFO capacity, nonblocking push and wrap.
It is intentionally a small supported subset, not a general Pico emulator.
Unknown assembly/directives fail loudly rather than being ignored. A small
independent instruction self-check runs before the suite.

The output-PIO tests take duration/pattern words generated by the actual C
firmware for two adjacent high entries. They then remove producer starvation to
isolate PIO behaviour. The clock-PIO test delays the CPU response read to expose
old RX samples; the high-level fake-clock tests cannot discover that bug.

## Coverage and policy

The tests include:

- All 256 sequence addresses and masks; zero, one, four and large 24-bit durations.
- Complete response packets, undefined opcodes, camera-index validation, PIV
  network order, fixed source readback and no-response command framing.
- All production Pianolas files, including several exposure substitutions for
  Bad Nauheim. Their 10 us entries round to four ticks (10.24 us).
- Representative calibration and self-trigger uploads from CameraTiming.mm,
  including the extra excluded entry uploaded after SetEnd. Those two scenarios
  are hand-authored fixtures, not automatic execution of Objective-C; keep them
  in sync if the host programming routines change.
- Full-length one-shots, repeat sections, single-entry loops and a boundary matrix
  spanning lengths 1, 2, 17 and 256 and repeat positions 0/middle/final.
- Thousands of deterministic clock and live-mutation cases. Seeds are fixed in
  the test source and failure output identifies the event for live mutations.
- Scheduled starts across wrap/signed boundaries, replacement, cancellation,
  missed firing windows, stops, restart and hard reset.
- Bursty uploads, incomplete reception, queue rejection, debug record/padding
  integrity and invalid repeat ranges.

Live-edit safety tests do not prescribe immediate versus deferred versus ignored
edits. They check bounded state/index/FIFO behaviour and continued command service.
Other tests deliberately require stop/restart/pending-start replacement to work.
The stop-queued test requires that a queued successor cannot execute after stop;
it does not demand an arbitrary all-low output policy. Reset must clear repeat
and pending configuration. Invalid repeat ranges must not leave playback stuck
active indefinitely; rejecting/clamping/stopping are all possible solutions.

The refill timing experiments use **requested durations and an assumed four-tick
manager servicing interval**, with an adversarial phase. A long entry surrounds
one short entry. The four-tick case and one-tick case illustrate buffer margin;
they do not measure the CPU or certify a minimum supported pulse width. These
are separate from the assembly timing tests, which count the existing PIO's
actual instruction cycles. Timing conversions assume the configured 150 MHz
system clock; external oscillator error is unmeasured.

## Limits and occasional hardware validation

This suite does not measure CPU throughput, instruction-cache misses, DMA/bus
arbitration, real spinlock contention, IRQ scheduling, UART FIFO rates, electrical
levels or actual pin-edge timing. The native test scheduler does not reproduce
all multicore interleavings. A passing core-ownership check is not a concurrency
proof. A passing fake-clock test does not validate the physical clock.

No DMA fake is implemented. If DMA is adopted, retain the behavioural tests and
extend the peripheral boundary for real descriptor construction, pacing,
completion, chaining and abort. Do not substitute a copied playback algorithm.

Occasional hardware checks should establish:

1. Four-tick timing for isolated and consecutive short entries, including repeat
   boundaries, under idle and UART load. Check both edge spacing and pulse width.
2. Continuous high across adjacent entries and final-state retention.
3. Stop/restart with active and queued work, and scheduled-start latency/jitter.
4. Clock freshness, wrap and absolute tick rate against an independent reference.
5. If DMA is introduced, shortest loops, channel abort/restart and no stale work.

A second Pico capture program can automate most digital checks without a routine
logic-analyser session. Verify electrical levels before wiring BNC signals to
Pico inputs. Oscilloscope checks are appropriate for edge quality and load effects.

## Adding tests

Add native tests to the table in `native/test_firmware.c`; send real protocol bytes
through `send_packet`, independently specify expected patterns/durations, and use
bounded stepping. Prefix a native test name with `timing_` to classify it as a
timing diagnostic. Add assembly checks through `pio_tests()` in `run_tests.py`.

Keep assertions tied to the protocol or safety requirements, not implementation
variable names where an observable result is available. Explain any permitted
alternatives. Every fake-peripheral extension should remain smaller than the
hardware behaviour it replaces and should fail on unsupported operations.
