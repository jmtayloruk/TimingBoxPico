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
WARN [TIMING] pio_entry_spacing — Timing / one- and four-tick entry spacing
     Expected entry writes at [0, 32, 160] PIO cycles, ...
```

Existing firmware bugs are deliberately ordinary failing tests, not expected
failures which turn the suite green. Fixing them should turn their tests green.
Compatibility tests cover unsupported source IDs and PIV-enable being a no-op.
Routing tests cover queued and held masks, clock, status and constant overrides,
and the shared clock phase with a manually advanced microsecond timer.

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
blocking call may abandon an allocation. Any `malloc` calls from the included firmware are initially
poisoned with `0xA5`, making uninitialised-output defects reproducible without
MemorySanitizer. The current UART path uses fixed buffers rather than allocation. Poisoning does not itself establish that all uninitialised reads
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
  The actual `core0Step()` is stepped independently from `pianolaManagerStep()`.
  Startup tests verify polling runs and UART interrupts are disabled. TX can be
  paused while RX progresses; full command/reply queues exercise retention.

The generated native PIO headers contain the **actual C SDK initialisers embedded
in the .pio files** and the parsed wrap-target offsets. Startup tests run the real
initialisation and, where required, its first core-0 foreground iteration. Settings
run against the strict shim, which rejects a SET width greater than five. The current pin-configuration test also
checks that unrelated GPIOs are not assigned or driven.

`pio_model.py` interprets the actual assembly in the .pio files, including branch
post-decrement, instruction delays, FIFO capacity, nonblocking push and wrap.
It is intentionally a small supported subset, not a general Pico emulator.
Unknown assembly/directives fail loudly rather than being ignored. A small
independent instruction self-check runs before the suite.

The output-PIO tests take duration/pattern words generated by the actual C
firmware for short sequences, including adjacent patterns sharing a high bit,
minimum durations and a nonzero repeat section. Refilling promptly isolates PIO
instruction timing from CPU starvation; expected transitions are specified
independently. Clock-PIO tests verify one fresh sample per request and twelve-cycle
ticks on normal, response and wrap paths, including consecutive requests and
requests arriving at each instruction boundary. The high-level fake-clock tests cannot
establish these assembly properties.

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
- Scheduled starts across wrap/signed boundaries, replacement, rejection,
  cancellation and firing after a missed deadline, while preserving active
  playback until the scheduled restart. Stops, restart and hard reset.
- Bursty uploads, incomplete reception, queue rejection, debug record/padding
  integrity and invalid repeat ranges.

Live-edit safety tests do not prescribe immediate versus deferred versus ignored
edits. They check bounded state/index/FIFO behaviour and continued command service.
Other tests deliberately require stop/restart/pending-start replacement to work.
Stop must cancel active/queued work, rewind to entry zero, apply its pattern and
hold it indefinitely with the output SM disabled. This includes a nonzero entry
zero and stop after natural completion. It deliberately differs from XMOS, which
retains the last applied pianola pattern. Hard reset must cancel active/queued
work, drive outputs low, clear stored entries and reset sequence/repeat/pending,
camera-clock and PIV settings, without resetting the independent clock. An invalid
repeat range finishes the current pass naturally, retaining its final pattern
and stored repeat settings; it does not invoke the entry-zero-freezing stop
command. Validation is at the boundary, allowing repeatFrom before final during
upload and handling live invalidation.

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
   For a two-channel capture, repeat mask `0x01` for one tick and mask `0x03`
   for four ticks. Bit 0 should stay high throughout and bit 1 should alternate
   2.56 µs low and 10.24 µs high. Capture both idle and under UART load.
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

## Connected-Pico serial stress tests

A separate script exercises the real serial connection. It does not use or
replace the native peripheral shim, install firmware, or require Python packages.
It uses the standard-library POSIX tty interface on macOS/Linux at 115200 8N1.

From the TimingBoxPico directory, first list candidate ports (no port is opened):

```sh
python3 tests/hardware_stress.py --list-ports
```

Close the SPIM application/other serial clients, then choose the
FTDI device. The script claims exclusive tty access where supported. This is a
test of a device you can reprogram: it replaces and runs its pianola. The default
pattern uses bit 4. The script temporarily routes bit 4 to the Board LED (GP25)
if the LED defaults to Pianola running, and restores that route during cleanup
if the serial connection is healthy. Other bits are low in the patterns.
Disconnect attached equipment if its mapping is uncertain. The firmware version
and LED route are checked before running patterns. These checks cannot establish
physical wiring or uniquely identify the device.

Omit `--port` to see a numbered list of all `/dev/cu.usbserial*` ports and choose
one interactively (even if only one is available). Enter `q` to cancel without
opening a device. If no matching ports are found, an error suggests connecting
the device or using `--port`. Unattended runs should always specify `--port`.

```sh
python3 tests/hardware_stress.py --interactive
python3 tests/hardware_stress.py --port /dev/cu.usbserial-XXXX
python3 tests/hardware_stress.py --port /dev/cu.usbserial-XXXX --seconds 60 --json /tmp/pico-stress.json
python3 tests/hardware_stress.py --port /dev/cu.usbserial-XXXX --interactive
```

The automatic cases check:

- A reset/reframing train followed by a firmware-version query.
- Isolated version/time queries, with **no wake-up command or retry** while a
  response is pending. A reply stranded until the next RX interrupt fails.
- Exact reply lengths and a short quiet interval to detect unsolicited bytes.
- Clock advancement against broad host-time bounds. USB/OS latency prevents this
  being an absolute-frequency calibration; suspicious timing is a warning.
- Source readback for the chosen logical LED output, including temporary Board LED routing.
- Back-to-back no-response uploads of 3, 32 and 256 entries, followed by run,
  stop and query commands. The protocol has no sequence readback: this checks
  subsequent replies and responsiveness, not complete upload contents.
- Acceptance of scheduled/replacement starts, restart and stop, followed by
  queries. Physical execution/cancellation is not established by these replies.
- A complete upload packet fragmented across two writes, 20 ms apart. This does
  not deliberately abandon a packet, and does not establish nonblocking service.
- Seeded mixed queries/starts/stops while repeating playback is requested. No
  concurrent writers or interleaved packets are used. A seed reproduces the
  chosen actions, not nondeterministic USB timing.

Default stress duration is 10 seconds. `--seconds` changes that phase only; the
setup/case/cleanup time is additional. `--timeout` bounds each reply/write
(default 0.5 s). Communications checks use a generous **100 ms** scheduling lead
by default (`--lead-ms` overrides it). They test command service rather than
short-lead tolerance. An explicitly shorter override can still expire because of
host/USB delay; the error prints target and reported time.

Use `--short-lead-ms 10` to add a separate opt-in TIMING diagnostic. A well-formed
rejection whose returned sample shows the target already expired is WARN rather
than a communications failure. Acceptance is PASS [TIMING] for that exchange only;
the request is then cancelled. A rejection of a still-future target, a malformed
reply, timeout or failed follow-up stop/query remains a FUNCTIONAL failure. This
is acceptance timing across the host/USB/firmware path, not output-edge timing.
There are no retries to hide the initial outcome.

```sh
python3 tests/hardware_stress.py --short-lead-ms 10
python3 tests/hardware_stress.py --short-lead-ms 10 --strict-timing
```

 `--strict-timing` makes timing warnings affect exit status. Reply latency
statistics are host/USB round trips, not output latency. The JSON report includes
case results, all transmitted/received bytes, timestamps, seed and cleanup result.

A framing error, absent/short response, or transport error stops further cases:
remaining cases are SKIP, rather than interpreting later bytes at uncertain
boundaries. There is no automatic retry that might hide the defect. Cleanup
attempts an explicit low sequence then stop when the stream remains trusted;
with a broken stream it only attempts stop. Neither path proves output levels.
Check/power-cycle the device if cleanup fails. Ctrl-C attempts cleanup and returns
130. Other failing cases return 1, argument errors return 2, and otherwise the
script returns 0. Running the existing firmware may expose the known UART bugs.

### Optional human LED observations

`--interactive` adds five separately labelled HUMAN checks, with y/n/skip answers:

Before **each** check, the script stops previous activity, executes an explicit
low pattern, prints the complete instructions, and waits for Return. The LED
should be OFF while you read. Enter `n` if the starting state is wrong or `s` to
skip without starting. Once you press Return, watch only the LED: there are no
mid-window questions or new instructions. After viewing, the script stops
repeating playback and asks retrospective questions. One-shot checks leave the
final level visible through the question.

1. A burst-uploaded, scheduled 1 Hz pattern, ON for roughly 25% of each cycle.
   After Return there is about half a second OFF, then five seconds of flashing.
   This visual check uses a fixed 500 ms lead, independent of `--lead-ms`, so USB
   serial latency does not normally prevent the waveform being observed. It is
   not a test of accepting a short-lead request.
2. A fresh blinking pattern for four seconds, followed by stop; observe three
   more seconds of steady ON output: entry zero is ON, so StopAndReset must freeze ON.
3. Replace a 2-second pending start with a 6-second one. Observe the whole
   ten-second window without looking back at the terminal: OFF for about six
   seconds, then blinking. Both questions are asked afterwards, so the prompt
   itself cannot obscure whether the original start was cancelled.
4. A **one-shot**, not repeating, four-second ON entry followed by OFF. Restart
   after three seconds: expect about seven seconds of continuous ON, then OFF
   for the rest of the eleven-second viewing window. An early OFF near four
   seconds, a gap or relighting is a failure. This deliberately replaces the
   ambiguous repeating two-seconds-ON/two-seconds-OFF check.
5. A one-shot with one second OFF followed by a one-tick ON entry. The LED
   should remain ON for the rest of the observation and through the question.

Watch the **Board LED** for these checks. Interactive mode uses the temporary
bit-4 Board LED mapping. Its viewing windows total about
38 seconds, plus serial exchanges, preparation and answer time. 'n' records a functional human-observed failure, 's'
records SKIP, and EOF records a failed observation. Human passes supplement,
rather than upgrade, automated serial passes. They are useful for gross waveform/
control errors, but cannot reveal microsecond glitches, exact duty cycle, clock
accuracy or short-entry starvation. An analyser/second Pico is still appropriate
for those.

### Offline tests of the hardware runner

These do **not** connect to a Pico. They validate the script's own protocol
construction, timeouts/no-retry behaviour, fragmented/extra/short replies,
classification, JSON report, cleanup and POSIX transport through a pseudo-terminal:

```sh
python3 -B -m unittest discover -s tests -p 'test_hardware_stress.py' -v
```

## UART ownership implementation

Core 0 exclusively receives/transmits UART bytes from `core0Step()` in its main
loop. There is no UART IRQ handler and core 1 never touches UART. Each step handles
at most 32 RX and 32 TX bytes; incomplete packets are retained without waiting.
Command and response queues each hold 32 records. A complete command is retained
if its queue is full. Core 1 similarly retains a reply if responseQ is full and
continues playback stepping while waiting to publish it. No UART-path heap
allocation is needed. Debug dumps travel as ordered queue requests and are
transmitted incrementally, with zero-filled unused records.

Backpressure is finite: this does not make an unbounded host stream lossless.
If core 1 ceases making progress long enough, the hardware UART RX FIFO can still
overrun; no RTS/CTS or new protocol acknowledgement has been introduced. Likewise
an abandoned partial packet is retained, not silently timed out. A hardware
watchdog and a more explicit serial recovery protocol are separate work.

Focused native checks:

```sh
python3 tests/run_tests.py --filter uart
python3 tests/run_tests.py --filter fragmented_packet
python3 tests/run_tests.py --filter upload_burst
```

These are deterministic checks of the actual polling/queue code, not a proof
about physical interrupt latency. Rerun the connected burst test after flashing.

## Stop/restart cancellation tests

`cancelPinDriver()` disables only the output SM, clears its FIFOs, restarts its
internal delay/shift state, and forces a jump to the generated wrap-target entry
point. Stop then forces a PULL/OUT of entry zero's pattern while halted and leaves
the SM disabled. Start preloads the first duration/pattern pair before enabling.
Cancellation itself does not add instructions to the normal playback path.
The output PIO separately implements exact entry spacing when fed in time; the
independent clock SM is never cancelled or restarted by stop/restart. Hard reset
uses the same halted-driver mask operation with zero, rather than entry zero.

Tests exercise 200 stop and 50 restart combinations of PC, FIFO occupancy
(including half-consumed pairs), stale delay/shift state and output pattern.
The fake peripheral now records enable/clear/restart/forced instructions, preserves
PC on hardware restart, models the forced PULL/OUT/JMP subset, and separately
records status LEDs and sequence-controlled outputs. These are API/state checks,
not a cycle-accurate CPU/peripheral race simulation. Existing instruction-model
and occasional physical capture checks remain complementary.

Additional tests cover repeated stop/start, restart during repetition, cancellation
at scheduled firing (not at arming), stop when already ready, and final-entry
consumption versus completion. The final READY transition waits until FIFO-empty
and the PC has returned to the output program's wait-loop entry point; arbitrary
new PIO control flow would need this completion condition reviewed.

```sh
python3 tests/run_tests.py --filter control
```

## Checking timebase agreement

`clock_divider_agreement` checks that both configured timebases take exactly 384
system-clock cycles per tick: twelve clock-PIO cycles at integer divider 32 and
32 output-PIO cycles at integer divider 12. The instruction-model tests separately
check the clock's normal, sample-response and wrap paths. These checks establish
matching configured scales, not measured oscillator accuracy or CPU latency.
See [Timing model](../README.md#timing-model) for the timing guarantees and limits.

The previous 40.69 ppm divider-representation warning should now be absent.
The separate FIFO-refill diagnostic can still warn about an assumed slow CPU
servicing interval; the clock change does not address FIFO starvation.

## Invalid repeat-range validation

At a repeat boundary, `repeatFrom > final` prevents another pass from being
queued. Any in-flight work finishes, then playback becomes READY with the last
pattern held. Stored configuration is retained: correcting the range allows a
subsequent run to repeat, and correcting it before the current boundary is
resolved can allow playback to continue. No PIO timing or instructions change.

Native tests cover invalid final+1/255 positions, varied sequence sizes, valid
repeat-before-final upload order, changing repeatFrom while active, shortening
final during repetition, repairing the range and continued command service.
Existing valid-repeat boundary tests include the full 256-entry range.
